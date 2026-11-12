// Evaluation only: request solver contact data; never write pose, forces or controls.
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <cmath>
#include <unistd.h>
#include <sdf/Box.hh>
namespace swan_eval {
using J=nlohmann::json;
using namespace gz::sim;
class Contacts:public System,public ISystemConfigure,public ISystemPreUpdate,public ISystemPostUpdate {
 std::string out; std::ofstream events; std::set<Entity> chair;
 std::map<Entity,std::string> names,objects; std::set<std::string> support;
 std::set<std::string> active; uint64_t count=0,steps=0,excluded=0; double last=-1;
 bool validation=false; bool armed=false; J inventory=J::array();
 std::string named(Entity e,const EntityComponentManager &ecm) {
  std::string result; while(e!=kNullEntity&&!ecm.Component<components::World>(e)){
   auto n=ecm.Component<components::Name>(e);if(n)result=n->Data()+(result.empty()?"":"::"+result); e=ecm.ParentEntity(e);
  }return result;
 }
 std::string object(Entity e,const EntityComponentManager &ecm) {
   Entity highest=e;
   while(e!=kNullEntity) {if(ecm.Component<components::Model>(e))highest=e; e=ecm.ParentEntity(e);}
   return named(highest,ecm);
 }
 void write(const J &j){events<<j.dump()<<'\n';events.flush();}
 public:void Configure(const Entity&,const std::shared_ptr<const sdf::Element>&s,EntityComponentManager&,EventManager&)override {
  validation=s->Get<bool>("validation",false).first; out=s->Get<std::string>("output"); std::filesystem::create_directories(out);
  events.open(out+"/contacts.jsonl",std::ios::app);
  auto el=s->FindElement("support");
  while(el){support.insert(el->Get<std::string>());el=el->GetNextElement("support");}
 }
 void PreUpdate(const UpdateInfo&,EntityComponentManager&ecm)override {
  if(!chair.empty())return;
  ecm.Each<components::Collision>([&](Entity e,const components::Collision*) {
   auto name=named(e,ecm); names[e]=name;objects[e]=object(e,ecm);
   if(objects[e]=="wheelchair") {
    chair.insert(e);inventory.push_back(name);
    if(!ecm.Component<components::ContactSensorData>(e))ecm.CreateComponent(e,components::ContactSensorData());
   } return true;});
  if(!chair.empty())write({{"type","inventory"},{"collisions",inventory},{"support",support}});
 }
 void PostUpdate(const UpdateInfo&i,const EntityComponentManager&ecm)override {
  if(i.paused||chair.empty())return;
  double t=std::chrono::duration<double>(i.simTime).count(); steps++;
  std::map<std::string,J> current;
  for(auto e:chair){
   auto data=ecm.Component<components::ContactSensorData>(e); if(!data)continue;
   for(const auto &c:data->Data().contact()) {
    Entity a=c.collision1().id(),b=c.collision2().id();
    bool ca=chair.count(a),cb=chair.count(b); if(ca==cb)continue;
    Entity other=ca?b:a,own=ca?a:b;
    std::string name=names.count(other)?names.at(other):named(other,ecm);
    std::string obj=objects.count(other)?objects.at(other):object(other,ecm);
    bool floor=support.count(name)>0 && c.normal_size()>0;
    for(const auto &n:c.normal())floor=floor&&std::abs(n.z())>.70;
    // At a coplanar box seam DART can report a horizontal edge normal, even
    // though a rolling wheel only touches the top plane (micrometre penetration).
    // Exclude only listed support boxes, wheel shapes, and all points within
    // 1 mm of that box's top surface. A real vertical step below its top counts.
    if(!floor && support.count(name) && names[own].find("_wheel::")!=std::string::npos && c.position_size()>0){
     auto geometry=ecm.Component<components::Geometry>(other);
     if(geometry && geometry->Data().BoxShape()){
      auto transform=worldPose(other,ecm).Inverse();
      double top=geometry->Data().BoxShape()->Size().Z()*.5;
      floor=true;
      for(const auto &p:c.position()){
       auto local=transform.CoordPositionAdd(gz::math::Vector3d(p.x(),p.y(),p.z()));
       floor=floor&&std::abs(local.Z()-top)<=.001;
      }
     }
    }
    if(floor){excluded++;continue;}
    if(!current.count(obj))current[obj]={{"object",obj},{"chair_collisions",J::array()},{"external_collisions",J::array()},{"points",J::array()},{"normals",J::array()},{"depths",J::array()}};
    auto &v=current[obj];v["chair_collisions"].push_back(names[own]);v["external_collisions"].push_back(name);
    for(const auto&p:c.position())v["points"].push_back({p.x(),p.y(),p.z()});
    for(const auto&n:c.normal())v["normals"].push_back({n.x(),n.y(),n.z()});
    for(auto depth:c.depth())v["depths"].push_back(depth);
   }
  }
  // Arm after preflight. A fresh simulator is used for every attempt.
  if(!armed&&std::filesystem::exists(out+"/armed")){armed=true;active.clear();count=0;write({{"type","armed"},{"sim_time",t}});}
  std::set<std::string> now;
  for(auto &[obj,v]:current){now.insert(obj);if(!active.count(obj)){
    if(armed)count++;v["type"]="contact_begin";v["sim_time"]=t;v["count"]=count;v["armed"]=armed;write(v);
  }}
  for(auto &obj:active)if(!now.count(obj))write({{"type","contact_end"},{"object",obj},{"sim_time",t},{"count",count},{"armed",armed}});
  active=now;
  if(t-last>=.1||last<0){
   J j={{"sim_time",t},{"physics_steps",steps},{"collision_count",count},{"armed",armed},{"registered_collisions",inventory},{"active_objects",active},{"excluded_support_pairs",excluded}};
   if(validation){J poses=J::object();for(auto e:chair){auto p=worldPose(e,ecm);poses[names[e]]={p.Pos().X(),p.Pos().Y(),p.Pos().Z(),p.Rot().Roll(),p.Rot().Pitch(),p.Rot().Yaw()};}j["collision_poses"]=poses;}
   std::string tmp=out+"/contacts_state.json.tmp";{std::ofstream f(tmp);f<<j.dump()<<'\n';}std::filesystem::rename(tmp,out+"/contacts_state.json");last=t;
  }
 }
};
}
GZ_ADD_PLUGIN(swan_eval::Contacts,gz::sim::System,swan_eval::Contacts::ISystemConfigure,swan_eval::Contacts::ISystemPreUpdate,swan_eval::Contacts::ISystemPostUpdate)
