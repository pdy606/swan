"""Add read-only solver contact instrumentation to a separate derived world."""
from pathlib import Path
from urllib.parse import unquote, urlparse
import json
import xml.etree.ElementTree as ET
from .derived_world import canonical

SUPPORT_MODELS = {'ground_environment','ground_plane','ground','road','sidewalk_left',
 'sidewalk_right','lobby_floor','integration_parking_market_path','integration_city_access_ramp',
 'entrance_sidewalk_west','entrance_sidewalk_market','integration_city_branch'}
RAMP_LINKS = {'entrance_landing','ramp_floor','bottom_sidewalk','curb_ramp'}


def chair_collisions(world):
    """Scoped names of every collision in the spawned wheelchair model (occupant dummy included).

    Must match the contact plugin inventory: the plugin registers all collisions whose
    top-level model is ``wheelchair``, so the expected set follows the model file
    instead of a fixed count.
    """
    includes=[i for i in world.findall('./world/include') if i.findtext('name')=='wheelchair']
    if len(includes)!=1:raise ValueError('Expected exactly one wheelchair include')
    uri=includes[0].findtext('uri') or ''
    if not uri.startswith('file://'):raise ValueError('Wheelchair include must be a derived file:// model')
    path=Path(unquote(urlparse(uri).path))
    path=path/'model.sdf' if path.is_dir() else path
    model=ET.parse(path).getroot().find('./model')
    names=['::'.join(('wheelchair',link.get('name'),collision.get('name')))
           for link in model.findall('link') for collision in link.findall('collision')]
    if not names or len(set(names))!=len(names):raise ValueError('Wheelchair collisions missing or duplicated')
    return names


def instrument(world_path, root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    if not output.is_relative_to(root):
        raise ValueError('Evaluation files must stay inside AI package')
    library=root/'runtime/contact_build/libswan_evaluation_contacts.so'
    if not library.exists():
        raise ValueError('Build evaluation/CMakeLists.txt first')
    output.mkdir(parents=True,exist_ok=True)
    original=ET.parse(world_path).getroot()
    tree=ET.fromstring(ET.tostring(original))
    plugin=ET.SubElement(tree.find('world'),'plugin',filename=str(library),name='swan_eval::Contacts')
    ET.SubElement(plugin,'output').text=str(output)
    surfaces=[];chair=chair_collisions(tree)
    for model in tree.findall('./world/model'):
        for link in model.findall('link'):
            if (model.get('name') in SUPPORT_MODELS or
                model.get('name')=='wheelchair_ramp' and link.get('name') in RAMP_LINKS or
                model.get('name')=='parking_lot' and link.get('name')=='asphalt'):
                for collision in link.findall('collision'):
                    name='::'.join((model.get('name'),link.get('name'),collision.get('name')))
                    surfaces.append(name)
                    ET.SubElement(plugin,'support').text=name
    path=output/'evaluation_world.sdf'
    ET.ElementTree(tree).write(path,encoding='utf-8',xml_declaration=True)
    check=ET.parse(path).getroot()
    check.find('world').remove(check.find("./world/plugin[@name='swan_eval::Contacts']"))
    if canonical(check)!=canonical(original):
        raise ValueError('Instrumentation changed world semantics')
    (output/'evaluation_manifest.json').write_text(json.dumps({
        'base_world':str(world_path),'world':str(path),'support_collisions':surfaces,
        'chair_collisions':chair,
        'support_normal_abs_z_min':.7,'physics_unchanged':True,
        'counting':'one event per external top-level model; exact solver-step release',
        'data_path':'solver ContactSensorData; no control subscriptions/publications'},indent=2)+'\n')
    return path
