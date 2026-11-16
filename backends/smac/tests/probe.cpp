#include <nav2_smac_planner/collision_checker.hpp>
#include <nav2_smac_planner/node_hybrid.hpp>
#include <nav2_smac_planner/smac_planner_hybrid.hpp>
#include <nav2_smac_planner/utils.hpp>
#include <nav2_smac_planner/analytic_expansion.hpp>
#include <pluginlib/class_loader.hpp>
#ifdef PATCHED
#include <nav2_smac_planner/output_validation.hpp>
#endif
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <limits>
using J=nlohmann::json;
using namespace nav2_smac_planner;
void require(bool ok,const char*msg){if(!ok)throw std::runtime_error(msg);}
nav2_costmap_2d::Footprint rect(double l,double r,double w){nav2_costmap_2d::Footprint f;for(auto xy:{std::pair<double,double>{l,w},{l,-w},{r,-w},{r,w}}){geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;f.push_back(p);}return f;}
geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation=getWorldOrientation(yaw);return p;}
int main(int argc,char**argv){rclcpp::init(argc,argv);int exit_code=0;
try{
 auto n=std::make_shared<rclcpp_lifecycle::LifecycleNode>("smac_regression");
 J input;std::ifstream(argv[1])>>input;auto g=input["grid"];
 nav2_costmap_2d::Costmap2D map(g["width"],g["height"],g["resolution"],g["origin"][0],g["origin"][1],0);
 for(size_t i=0;i<g["data"].size();i++)map.getCharMap()[i]=g["data"][i].get<int>();
 nav2_costmap_2d::Footprint fp;for(auto xy:input["footprint"]){geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];fp.push_back(p);}
 GridCollisionChecker checker(nullptr,72,n);checker.setCostmap(&map);checker.setFootprint(fp,false,0);
 nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> oracle(&map);
 int mismatch=0,total=0;
 for(auto s:input["samples"]){float mx,my;map.worldToMapContinuous(s[0],s[1],mx,my);int bin=static_cast<int>(std::round(s[2].get<double>()/(2*M_PI/72)));bin=(bin%72+72)%72;auto p=getWorldCoords(mx,my,&map);double yaw=checker.getPrecomputedAngles()[bin];bool collision=checker.inCollision(mx,my,bin,false);double cost=oracle.footprintCostAtPose(p.position.x,p.position.y,yaw,fp);if(!collision&&cost==254)mismatch++;total++;}
 std::cout<<J({{"test","frozen_map_poses"},{"samples",total},{"free_but_output_lethal",mismatch}}).dump()<<std::endl;
#ifdef PATCHED
 require(mismatch==0,"continuous coordinate mismatch");
#endif
 // Same graph node, different continuous position, then changed map. No sensors.
 nav2_costmap_2d::Costmap2D small(30,30,1,0,0,0);for(unsigned y=0;y<30;y++)small.setCost(11,y,254);
 GridCollisionChecker cc(nullptr,72,n);cc.setCostmap(&small);cc.setFootprint(rect(.3,-.3,.3),false,0);
 NodeHybrid node(0);node.setPose({10.1f,10.5f,0.f});bool a=node.isNodeValid(false,&cc);
 node.setPose({10.9f,10.5f,0.f});bool b=node.isNodeValid(false,&cc);
 node.setPose({10.1f,10.5f,0.f});bool c=node.isNodeValid(false,&cc);
 std::cout<<J({{"test","subcell_cache"},{"free_A",a},{"free_B",b},{"free_A_again",c}}).dump()<<std::endl;
#ifdef PATCHED
 require(a&&!b&&c,"pose cache reuse");
 NodeHybrid * node_ptr = &node; auto coords=node.pose;
 AnalyticExpansion<NodeHybrid>::AnalyticExpansionNode snapshot(node_ptr,coords,coords);
 const float saved_cost=snapshot.cost;
 small.setCost(10,10,254);require(!node.isNodeValid(false,&cc),"map update cache reuse");
 require(snapshot.cost==saved_cost && node.getCost()!=saved_cost,"analytic cost snapshot mutated");
 for(float bad:{-1.f,30.f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}){require(cc.inCollision(bad,10,0,false),"x boundary");require(cc.inCollision(10,bad,0,false),"y boundary");require(cc.inCollision(10,10,bad,false),"angle boundary");}
 require(cc.inCollision(900u,false),"linear index boundary");
 small.setCost(10,10,0);small.setCost(10,11,254);small.setCost(11,10,0);
 GridCollisionChecker circle(nullptr,72,n);circle.setCostmap(&small);circle.setFootprint({},true,1);
 require(!circle.inCollision(10.9f,10.1f,0,false),"wrong rounded center");
 small.setCost(10,10,255);require(circle.inCollision(10.2f,10.2f,0,false),"unknown closed");require(!circle.inCollision(10.2f,10.2f,0,true),"unknown allowed");
 small.setCost(10,10,0);require(cc.inCollision(.1f,15,0,false),"footprint outside map");
 for(double origin:{-3.14159265,1e6+.1234567}){nav2_costmap_2d::Costmap2D m(60,60,.05,origin,origin,0);auto q=getWorldCoords(20.7f,20.7f,&m);require(std::abs(q.position.x-(origin+double(20.7f)*.05))<1e-9,"origin precision");}
 nav_msgs::msg::Path crossing;crossing.poses={pose(9,15,0),pose(13,15,0)};
 require(!validOutputPath(crossing,&small,rect(.3,-.3,.3),false,false),"segment crossed obstacle");
 nav_msgs::msg::Path good;good.poses={pose(6,15,0),pose(8,15,0)};
 require(validOutputPath(good,&small,rect(.3,-.3,.3),false,false),"valid segment rejected");
 std::cout<<"{\"test\":\"boundary_unknown_origin_segments\",\"passed\":true}"<<std::endl;
#endif
 // Actual Hybrid plugin, using a frozen native costmap, not a mock planner.
 for(bool smooth:{false,true}){
  auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("frozen_costmap");
  cm->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  cm->set_parameter(rclcpp::Parameter("footprint",std::string("[[0.57,0.32],[0.57,-0.32],[-0.37,-0.32],[-0.37,0.32]]")));
  cm->set_parameter(rclcpp::Parameter("footprint_padding",0.0));
  cm->on_configure(rclcpp_lifecycle::State());auto m=cm->getCostmap();m->resizeMap(map.getSizeInCellsX(),map.getSizeInCellsY(),map.getResolution(),map.getOriginX(),map.getOriginY());std::copy(map.getCharMap(),map.getCharMap()+map.getSizeInCellsX()*map.getSizeInCellsY(),m->getCharMap());
  auto pn=std::make_shared<rclcpp_lifecycle::LifecycleNode>(smooth?"planner_smooth":"planner_raw");
  pn->declare_parameter("test.motion_model_for_search",std::string("REEDS_SHEPP"));
  pn->declare_parameter("test.minimum_turning_radius",.4);pn->declare_parameter("test.tolerance",0.0);
  pn->declare_parameter("test.smooth_path",smooth);pn->declare_parameter("test.allow_unknown",false);
  pn->declare_parameter("test.max_planning_time",1.0);pn->declare_parameter("test.lookup_table_size",10.0);
  pn->declare_parameter("test.analytic_expansion_max_length",2.0);pn->declare_parameter("test.analytic_expansion_max_cost",200.0);
  pn->declare_parameter("test.reverse_penalty",1.5);pn->declare_parameter("test.change_penalty",0.0);
  pn->declare_parameter("test.cost_penalty",2.0);pn->declare_parameter("test.cache_obstacle_heuristic",true);
 
#ifdef PATCHED
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  auto planner=loader.createUniqueInstance("navigation_smac_planner::SmacPlannerHybrid");
#else
  auto planner=std::make_unique<SmacPlannerHybrid>();
#endif
  planner->configure(pn,"test",nullptr,cm);planner->activate();
  J cases=J::array();
  std::vector<std::array<double,6>> routes={{{-1.15,2.2,1.57,0,-3,-1.57}},{{0,-3,-1.57,-1.15,2.2,1.57}},{{-1.15,2.2,1.57,0,-8,-1.57}},{{0,-3,-1.57,0,-8,-1.57}},{{0,-3,-1.57,0,-3,1.57}}};
  for(int repeat=0; repeat<5; ++repeat) for(auto route:routes){J out;out["repeat"]=repeat;out["route"]=route;auto begin=std::chrono::steady_clock::now();try{
    auto path=planner->createPlan(pose(route[0],route[1],route[2]),pose(route[3],route[4],route[5]),[]{return false;});
    int lethal=0;nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> exact(m);
    for(auto & q:path.poses)if(exact.footprintCostAtPose(q.pose.position.x,q.pose.position.y,tf2::getYaw(q.pose.orientation),cm->getRobotFootprint())==254)lethal++;
    out["success"]=true;out["poses"]=path.poses.size();out["lethal_poses"]=lethal;
#ifdef PATCHED
    require(lethal==0,"actual returned Path has lethal pose");require(validOutputPath(path,m,cm->getRobotFootprint(),false,false),"returned Path invalid");
#endif
   }catch(const nav2_core::PlannerException&e){out["success"]=false;out["reason"]=e.what();}

#ifdef PATCHED
   if (route[1] == route[4] && route[0] == route[3]) {require(!out["success"].get<bool>(),"unsafe rotation accepted");}
   else {require(out["success"].get<bool>(),"historical route regressed to no path");}
#endif
   out["seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();cases.push_back(out);
  }
  std::cout<<J({{"test","actual_planner"},{"smooth",smooth},{"cases",cases}}).dump()<<std::endl;

#ifdef PATCHED
  auto safe=planner->createPlan(pose(-1.15,2.2,1.57),pose(-1.15,2.2,1.57),[]{return false;});
  require(!safe.poses.empty(),"safe same-cell plan rejected");
  bool rejected=false;
  try {planner->createPlan(pose(-1.15,2.2,1.57),pose(std::numeric_limits<double>::quiet_NaN(),0,0),[]{return false;});}
  catch(const nav2_core::PlannerException&) {rejected=true;}
  require(rejected,"NaN goal accepted");
#endif
  planner->deactivate();planner->cleanup();planner.reset();cm->on_cleanup(rclcpp_lifecycle::State());
 }
}catch(std::exception&e){std::cerr<<"REGRESSION FAILED: "<<e.what()<<std::endl;exit_code=1;}
rclcpp::shutdown();return exit_code;}
