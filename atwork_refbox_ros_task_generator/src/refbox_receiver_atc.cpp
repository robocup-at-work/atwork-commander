#include <atwork_refbox_ros_task_generator/receiver_node_atc.h>

#include <std_srvs/Empty.h>
#include <atwork_refbox_ros_task_generator/objects.h>

#include <worldmodel_project/worldmodel_client.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <regex>
#include <unistd.h>
#include <math.h>
#include <tuple>
#include <iostream>
#include <unordered_map>

using namespace std;
using namespace atwork_refbox_ros;
using run = vector<array<int, 4>>;

template<typename T, size_t n>
ostream& operator<<(ostream& os, const vector<array<T,n>>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i];
  return os << endl;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<vector<T>>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i];
  return os << endl;
}

template<typename T, size_t n>
ostream& operator<<(ostream& os, const array<T,n>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i] << (i==vec.size()-1?"":" ");
  return os << endl;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<T>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i] << (i==vec.size()-1?"":" ");
  return os << endl;
}

void ReceiverNode::debugAll(string info, run &tasks) {
	cout<<info<<"\n===========================\n";
	cout<<"mAllTables\n"<<mAllTables;
	cout<<"validpicks\n"<<validpicks;
	cout<<"picksleft\n"<<picksleft;
	//cout<<"mTableTypes\n"<<mTableTypes;
	cout<<"container_ids\n"<<container_ids;
	cout<<"tasks\n"<<tasks;
}

/* obj, src, dst, cont
 * BNT: location, orientation [0,1,2,3], time [ca. 1-5s]
 * objects is the number of tasks
 */

// for every task set random object type
void ReceiverNode::generate_objects(run &tasks) {
	const size_t objects = mObjects.size();
	const size_t pptobjects = mPptObjects.size();
	const size_t count = tasks.size();
	srand(time(NULL));
	for(size_t i=0; i<count; ++i) {
		if(tasks.at(i).at(dst_id) == -1) {
			size_t obj = rand() % objects;
			tasks.at(i).at(obj_id) = mObjects.at(obj);
		}
		else {
			size_t pptobj = rand() % pptobjects;
			tasks.at(i).at(obj_id) = mPptObjects.at(pptobj);
		}
	}
}

// retuns an ascending order of int 0,..., n-1
std::vector<size_t> order (size_t n) {
	std::vector<size_t> up(n);
	for(size_t i=0; i<n; ++i) {
		up.at(i) = i;
	}
	return up;
}	

/* Input: integers n and k
 * Output: vector with k random entries between 0<= entry <n in ascending order
 */
void variation(vector<size_t> position, size_t k, vector<size_t> &vec_k, vector<size_t> &vec_n_k) {
	size_t n = position.size();
	vec_k.resize(k);
	vec_n_k.resize(n-k);
	size_t a;
	srand(time(NULL));
	for(size_t i=0; i<k; ++i) {
		a = rand() % (n-i);
		std::swap(position.at(i), position.at(n-a-1));
	}
	copy(position.begin(), position.begin()+k, vec_k.begin());
	copy(position.begin()+k, position.end(), vec_n_k.begin());
}

// if the second vector isn't needed, this function can be used as an adapter
void variation(vector<size_t> &positions, size_t k, vector<size_t> &vec_k) {
	vector<size_t> trash;
	variation(positions, k, vec_k, trash); 
}

size_t ReceiverNode::get_container_id(size_t table, size_t color) {
	// if there is already a container of this color on the same table
	for(size_t i=0; i<container_ids.size(); ++i) {
		if (container_ids.at(i).at(0) == table && container_ids.at(i).at(1) == color) {
			return container_ids.at(i).at(2);
		}
	}
	//else get new id from the worldmodel
	if(color == blue) {
		size_t new_id = insertContainer(-1, robotto_msgs::BLUE_CONTAINER, table);
		std::array<size_t, 3> id = {table, blue, new_id};
		container_ids.push_back(id);
		return id.at(2);
	}
	else if(color == red) {
		size_t new_id = insertContainer(-1, robotto_msgs::RED_CONTAINER, table);
		std::array<size_t, 3> id = {table, red, new_id};
		container_ids.push_back(id);
		return id.at(2);
	}
}

void ReceiverNode::initialize_mAllTables() {
	size_t size = mTables0.size() + mTables5.size() + mTables10.size()
					+ mTables15.size() + mConveyors.size()
					+ mPpts.size() + mShelfs.size();
	mAllTables.resize(size+1);
	auto end = copy(mTables0.begin()+1, mTables0.end(), mAllTables.begin());
	end = copy(mTables5.begin(), mTables5.end(), end);
	end = copy(mTables10.begin(), mTables10.end(), end);
	end = copy(mPpts.begin(), mPpts.end(), end);
	end = copy(mConveyors.begin(), mConveyors.end(), end);
	end = copy(mTables15.begin(), mTables15.end(), end);
	end = copy(mShelfs.begin(), mShelfs.end(), end);
}

// fills in all for every task every table where it isn't placed
void ReceiverNode::initialize_validpicks(run &tasks) {
	size_t objects = paramFinal["objects"];
	size_t tables = mAllTables.size();
	validpicks.resize(objects);
	for(size_t i=0; i<objects; ++i) {
		validpicks.at(i).resize(0);
		for(size_t j=0; j<tables; ++j) {
			if(mAllTables.at(j) != tasks.at(i).at(dst_id)) {
				validpicks.at(i).push_back(mAllTables.at(j));
			}
		}
	}	
}

void ReceiverNode::initialize_picksleft() {
	picksleft.resize(tabletypes);
	picksleft.at(tables0_id) = paramFinal["tables0"];
	picksleft.at(tables5_id) = paramFinal["tables5"];
	picksleft.at(tables10_id) = paramFinal["tables10"];
	picksleft.at(tables15_id) = paramFinal["tables15"];
	picksleft.at(conveyors_id) = paramFinal["pick_turntables"];
	picksleft.at(ppts_id) = paramFinal["pick_cavity_plattforms"];
	picksleft.at(shelfs_id) = paramFinal["pick_shelf"];
}


size_t ReceiverNode::write_types(size_t low, size_t up, size_t type) {
	for(size_t i=low; i<up; ++i) {
		mTableTypes[mAllTables.at(i)] = type;
	}
	return up;
}

void ReceiverNode::initialize_mTableTypes() {
	size_t low = 0;
	size_t up = mTables0.size();
	low = write_types(low, up, tables0_id);
	up += mTables5.size();
	low = write_types(low, up, tables5_id);
	up += mTables10.size();
	low = write_types(low, up, tables10_id);
	up += mTables15.size();
	low = write_types(low, up, tables15_id);
	up += mConveyors.size();
	low = write_types(low, up, conveyors_id);
	up += mPpts.size();
	low = write_types(low, up, ppts_id);
	up += mShelfs.size();
	low = write_types(low, up, shelfs_id);
}

size_t sum_vector(vector<size_t> & vec) {
	size_t sum = 0;
	for(size_t i=0; i<vec.size(); ++i) {
		sum += vec.at(i);
	}
	return sum;
}

size_t ReceiverNode::shortest_list(size_t &min) {
	min = validpicks.at(0).size();
	size_t minindex = 0;
	for(size_t i=0; i<validpicks.size(); ++i) {
		if(validpicks.at(i).size() != 0) {
			if(validpicks.at(i).size() < min) {
				min = validpicks.at(i).size();
				minindex = i;
			}
		}
	}
	return minindex;
}

//FEHLERBEHAFTET
void ReceiverNode::update_validpicks() {
	for(size_t i=0; i<tabletypes; ++i) {
		size_t size = validpicks.at(i).size();
    auto end = validpicks.at(i).end();
		for(auto j = validpicks.at(i).begin(); j<end; ++j) {
			size_t table = *j; // TABLE IST ID, KEIN INDEX
			size_t type = mTableTypes.at(table);
			if(picksleft.at(type) == 0) {
        iter_swap(j, --end);        // Remove element
			}
		}
    validpicks.at(i).erase(end, validpicks.at(i).end()); //Erase removed elements
	}
}

/* generate a BNT Task
 * 
 */
run ReceiverNode::generate_BNT() {
	run tasks;
	size_t waypoints = paramBNT["waypoints"];
	size_t tables = paramBNT["tables"];
	size_t places = tables + waypoints;
	srand(time(NULL));													// initialize the randomize function
	
	// validity check
	if (mWaypoints.size() == 0 && waypoints > 0) {throw 210;}			// No waypoints
	if (mTables.size() == 0 && tables > 0) {throw 211;}					// No tables
	
	std::vector<size_t>waypoint, table;
	std::vector<size_t>position = order(places);
	variation(position, waypoints, waypoint, table);					// split the tasks into waypoints and tables
	
	for(size_t i=0; i<places; ++i) {
		size_t orientation = rand() % 4;								// random orientation {0, 1, 2, 3}
		size_t time = (rand() % 4000) + 1001;							// time random between 1001 and 5000 ms
		tasks.at(i).at(obj_id) = orientation;
		tasks.at(i).at(dst_id) = time;
	}
	
	/* generate random waypoints for the waypoint tasks
	 * generate random tables for the table tasks
	 */
	for(size_t i=0; i<waypoint.size(); ++i) {
		size_t a = rand() % mWaypoints.size();
		tasks.at(waypoint.at(i)).at(src_id) = mWaypoints.at(a);
	}
	for(size_t i=0; i<table.size(); ++i) {
		size_t a = rand() % mTables.size();
		tasks.at(table.at(i)).at(dst_id) = mTables.at(a);
	}
	return tasks;
}

run ReceiverNode::generate_Final() {
	// initialize tasks
	run tasks(paramFinal["objects"], {-1, -1, -1, -1});
	size_t shelfs = mShelfs.size();
	size_t tables = mTables.size();
	size_t ppts = mPpts.size();
	size_t conveyors = mConveyors.size();
	size_t containers = paramFinal["B_Container"] + paramFinal["R_Container"];
	if(paramFinal["FlexibleHeight"] == true) {
		tabletypes = 8;
	} else {
		tabletypes = 7;
	}
		
	// check validity
	if (mObjects.size() == 0) {throw 220;}																			// No Objects
	if (shelfs == 0 && (paramFinal["pick_shelf"] > 0 || paramFinal["place_shelf"] > 0)) {throw 221;}				// No shelfpick/place without shelf
	if (tables == 0 && paramFinal["objects"] > paramFinal["pick_shelf"]) {throw 222;}								// No tables	
	if (shelfs + tables + conveyors + paramFinal["B_Container"] + paramFinal["R_Container"] <= 1) {throw 223;}		// pick = place
	if (paramContainerInShelf == true) {
		if (tables + shelfs == 0 && paramFinal["B_Container"] + paramFinal["R_Container"] > 0) {throw 224;}			// No tables, no shelfs, no conveyors, No container place in air
	}
	else if(tables == 0 && paramFinal["B_Container"] + paramFinal["R_Container"] > 0) {throw 225;}					// No tables No container place in air
	if (ppts == 0 && paramFinal["place_cavity_plattforms"] > 0) {throw 226;}										// No cavity plattforms
	if (conveyors == 0 && (paramFinal["pick_turntables"] > 0 || paramFinal["place_turntbales"] > 0)) {throw 227;}	// No conveyers
	
	/* select #place_shelf + #place_turntables + #ppts random tasks
	 *  select #place_shelf + #place_turntables of these 
	 *   select #place_shelf of these and set place to a random shelf
	 *   the rest of the places are on a random turntable
	 *  the rest of the places are into a random ppt
	 */
	std::vector<size_t>position = order(paramFinal["objects"]);
	std::vector<size_t>normalplace;
	std::vector<size_t>specialplace;
	std::vector<size_t>placeShelfTurntable;
	std::vector<size_t>placeShelf;
	std::vector<size_t>placeTurntable;
	std::vector<size_t>placePpt;
	std::vector<size_t>container;
	std::vector<size_t>b_container;
	std::vector<size_t>r_container;
	size_t specialplaces = paramFinal["place_shelf"] + paramFinal["place_turntables"] + paramFinal["place_cavity_plattforms"];
	size_t shelfTurntables = paramFinal["place_shelf"] + paramFinal["place_turntables"];
	size_t a;
	variation(position, specialplaces, specialplace, normalplace);
	variation(specialplace, shelfTurntables ,placeShelfTurntable, placePpt);
	variation(placeShelfTurntable, paramFinal["place_shelf"], placeShelf, placeTurntable);
	
	// write the Ppts as destinations to the tasks
	for(size_t i=0; i<placePpt.size(); ++i) {
		a = rand() % ppts;
		tasks.at(placePpt.at(i)).at(dst_id) = mPpts.at(a);
	}
	
	// generate the objects pptobjects seperately from the others
	generate_objects(tasks);
	
	// write all the other distinations to the tasks
	for(size_t i=0; i<placeShelf.size(); ++i) {
		a = rand() % shelfs;
		tasks.at(placeShelf.at(i)).at(dst_id) = mShelfs.at(a);
	}
	for(size_t i=0; i<placeTurntable.size(); ++i) {
		a = rand() % conveyors;
		tasks.at(placeTurntable.at(i)).at(dst_id) = mConveyors.at(a);
	}
	// PLACES NOCH BEARBEITEN KEINE PLACES, FALLS KEINE PICK VON DER TISCHHÖHE
	for(size_t i=0; i<normalplace.size(); ++i) {
		a = rand() % tables;
		tasks.at(normalplace.at(i)).at(dst_id) = mTables.at(a);
	}
	
	// collect all valid places to place a Container
	vector<size_t>valid_containerplaces = normalplace;
	vector<size_t>mBContainer, mRContainer;
	if(paramContainerInShelf == true) {
		copy(placeShelf.begin(), placeShelf.end(), back_inserter(valid_containerplaces));
	}
	if(paramContainerOnTurntable == true) {
		copy(placeTurntable.begin(), placeTurntable.end(), back_inserter(valid_containerplaces));
	}
	if(paramContainerOnPpt == true) {
		copy(placePpt.begin(), placePpt.end(), back_inserter(valid_containerplaces));
	}
	
	variation(valid_containerplaces, containers, container);
	variation(container, paramFinal["B_Container"], b_container, r_container);
	
	for(size_t i=0; i<paramFinal["B_Container"]; ++i) {
		size_t table = tasks.at(b_container.at(i)).at(dst_id);
		size_t blue_container_id = get_container_id(table, blue);
		tasks.at(b_container.at(i)).at(cont_id) = blue_container_id;	
	}
	for(size_t i=0; i<paramFinal["R_Container"]; ++i) {
		size_t table = tasks.at(r_container.at(i)).at(dst_id);
		size_t red_container_id = get_container_id(table, red);
		tasks.at(r_container.at(i)).at(cont_id) = red_container_id;
	}
	
	/* select #pick_shelf + #pick_turntables random tasks
	 * select #pick_shelf of these and set pick to a random shelf
	 * the rest of the picks are from a random turntable
	 * all of the other are set to a normalpick
	 */
	 
	/*
	size_t specialpicks = paramFinal["pick_shelf"] + paramFinal["pick_turntables"];
	std::vector<size_t>normalpick;
	std::vector<size_t>specialpick;
	std::vector<size_t>pickShelf, pickTurntable;
	position = order(paramFinal["objects"]);
	variation(position, specialpicks, specialpick, normalpick);
	variation(specialpick, paramFinal["pick_shelf"], pickShelf, pickTurntable);
	
	for(size_t i=0; i<pickShelf.size(); ++i) {
		a = rand() % shelfs;
		tasks.at(pickShelf.at(i)).at(src_id) = mShelfs.at(a);
	}
	for(size_t i=0; i<pickTurntable.size(); ++i) {
		a = rand() % conveyors;
		tasks.at(pickTurntable.at(i)).at(src_id) = mConveyors.at(a);
	}
	for(size_t i=0; i<normalpick.size(); ++i) {
		a = rand() % tables;
		tasks.at(normalpick.at(i)).at(src_id) = mTables.at(a);
	}
	*/
	debugAll("before initialisation", tasks);
	initialize_mAllTables();
	debugAll("after initialize_mAllTables", tasks);
	initialize_validpicks(tasks);
	debugAll("after initialize_validpicks", tasks);
	initialize_picksleft();
	debugAll("after initialize_picksleft", tasks);
	initialize_mTableTypes();
	debugAll("after initialize_mTableTypes", tasks);
	
	while(sum_vector(picksleft) > 0) {
		size_t min;
		size_t index = shortest_list(min);
    if (min == 0)
      throw 229;
		a = rand() % min;
		size_t table = validpicks.at(index).at(a);
		tasks.at(index).at(src_id) = mAllTables.at(table);
		validpicks.at(index).resize(0);
		size_t type = mTableTypes.at(table);
		--picksleft.at(type);
		update_validpicks();
	}
	
	return tasks;
}

run ReceiverNode::auto_task_creation() {
	switch (mdiscipline) {
		case 0	: return generate_BNT();
		case 7	: return generate_Final();
		default : throw 100;
	}
}


// VORLAGE
/*
static int toOrientation(NavigationTask::Orientation ori) {
  if(ori < NavigationTask::NORTH && ori > NavigationTask::WEST) {
    ROS_ERROR_STREAM("Invalid orientation specified by refbox: " << ori);
    return 0;
  } else
    return ori-1;
}
*/

/*
static double toTime(const Time& time) {
  return (double)time.sec()+(double)time.nsec()/1e9;
}
*/


// objects with unknown position get an default position stacked in front of the table
// NICHT ZU VERÄNDERN
/** \todo implement **/
string ReceiverNode::insertStackedTF(unsigned int table) {
  try {
    WorldmodelClient& wm = WorldmodelClient::getInstance();
    vector<worldmodel_project::Object> objs = wm.getObjects("/Workstation/"+to_string(table)+"/Objects");
    vector<string> tableTFNames = wm.getStrings("/Workstation/"+to_string(table)+"/SweepReference");
    worldmodel_project::TF tf;
    tf.frame_name         = to_string(table)+"Obj"+to_string(objs.size());
    tf.parent_frame       = tableTFNames.at(0);
    tf.timestamp          = ros::Time::now().toSec();
    tf.quality            = 0.0;
    tf.pose.position.y    = -0.2;
    tf.pose.position.z    = objs.size()*0.2;
    tf.pose.orientation.w = 1.0;
    ROS_INFO_STREAM("Insert new tf into world model: " << tf.parent_frame << " -> " << tf.frame_name << ": [(" << tf.pose.position.x << ", " << tf.pose.position.y << ", " << tf.pose.position.z << "), (" << tf.pose.orientation.x << ", " << tf.pose.orientation.y << ", " << tf.pose.orientation.z << ", " << tf.pose.orientation.w << ")]");
    wm.setTF("/TF/"+tf.frame_name+"/*", tf);
    return tf.frame_name;
  } catch(const out_of_range& e) {
    ROS_ERROR_STREAM("Sweep reference not set for table" << table);
    return "NULL";
  }
}

// similar to insertStackedTF for Container
// NICHT ZU VERÄNDERN
int ReceiverNode::insertContainer(int objType, int contType, unsigned int table) {

  WorldmodelClient& wm = WorldmodelClient::getInstance();
  ostringstream wsObjPath, wsTypePath;
  wsTypePath << "/Workstation/" << to_string(table) << "/Type";
  std::vector<std::string> wsType = wm.getStrings(wsTypePath.str());

  int type = -1;

  if(wsType.empty()){
    ROS_ERROR_STREAM("Table " << table << " has no type!");
    return -1;
  }

  if( wsType[0] == "PP") {
    switch(objType) {
      case(robotto_msgs::F20_20_G):
      case(robotto_msgs::F20_20_B): type = robotto_msgs::PPT_H_F20_20; break;
      case(robotto_msgs::S40_40_G):
      case(robotto_msgs::S40_40_B): type = robotto_msgs::PPT_H_S40_40; break;
      case(robotto_msgs::M20_100) : type = robotto_msgs::PPT_H_M20_100; break;
      case(robotto_msgs::M20)     : type = robotto_msgs::PPT_H_M20; break;
      case(robotto_msgs::M30)     : type = robotto_msgs::PPT_H_M30; break;
      case(robotto_msgs::R20)     : type = robotto_msgs::PPT_H_R20; break;
      default                     : ROS_ERROR_STREAM("Object Type " << objType << " to be placed on PPT " << table << " is unsupported");
                                    return -1;
    }
  }
  else {
    if(contType == -1 || (contType != robotto_msgs::BLUE_CONTAINER && contType != robotto_msgs::RED_CONTAINER)) {
      ROS_ERROR_STREAM("Container on Table " << table << " has invalid type " << contType);
      return -1;
    }
    type = contType;
  }

  wsObjPath << "/Workstation/" << to_string(table) << "/T_" << to_string(type);
  std::vector<worldmodel_project::Object> wsObjs = wm.getObjects(wsObjPath.str());
  if(!wsObjs.empty())
    return wsObjs.front().id;

  vector<string> objData = {
    to_string(type),         // Object  Type
    to_string(table),        // source table
    to_string(table),        // destination table
    "NULL",                  // destination container
    to_string(table),        // actual table
    "NULL",                  // inventory slot
    insertStackedTF(table)   // associated TF frame
  };

  ROS_INFO_STREAM("Insert " << robotto_msgs::objstr(type) << " on " << table);
  return wm.insert("/Object/New/*", objData );
}

// BEIBEHALTEN
// startet optimizer
void ReceiverNode::start() {
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;
  try {
    if(!start_service_.call(req, res))
      ROS_FATAL_STREAM("Start of task failed!");
  } catch(const runtime_error& e) {
    ROS_FATAL_STREAM("Cannot set task execution to true: " << e.what());
  }
}

// constructor
ReceiverNode::ReceiverNode(const ros::NodeHandle &nh):
    nh_(nh), seq_(0)
{
    readParameters();


	// WICHTIG, STARTET OPTIMIZER
    start_service_ = nh_.serviceClient<std_srvs::Empty> ("/robOTTO/optimizer/start");

    WorldmodelClient& wmc = WorldmodelClient::getInstance();
    wmc.setInt("/World/1/TaskRunning", 0);
    using WS = worldmodel_project::Workstation;
    using WP = std::string;
    std::vector<WS> tables = wmc.getWorkstations("/Workstation/*/*");
    sort(tables.begin(), tables.end(), [](const WS& a, const WS& b){return a.id < b.id;});
    unsigned int maxTableId = numeric_limits<unsigned int>::max();
    for(const worldmodel_project::Workstation& table : tables) {
      bool typeCorrect = false;
      maxTableId = table.id;
      if(table.type == "CB" ) {
        mConveyors.push_back(table.id);
        typeCorrect=true;
        ROS_INFO_STREAM("[RN] registered conveyor CB" << mConveyors.size() << " as T" << table.id);
      }
      if(table.type == "PP" ) {
        mPpts.push_back(table.id);
        typeCorrect=true;
        ROS_INFO_STREAM("[RN] registered precision placement PPT" << mPpts.size() << " as T" << table.id);
      }
      if(table.type == "TT" ) {
        mConveyors.push_back(table.id);
        typeCorrect=true;
        ROS_INFO_STREAM("[RN] registered round table CB" << mConveyors.size() << " as T" << table.id);
      }
      if(table.type == "Shelf" ) {
        mShelfs.push_back(table.id);
        typeCorrect=true;
        ROS_INFO_STREAM("[RN] registered shelf SH" << mShelfs.size() << " as T" << table.id);
      }
	if(table.type == "Normal" ) {
		mTables.push_back(table.id);
		// split into 4 vectors depending on the table heigth
		if(table.height == 0) {mTables0.push_back(table.id);}
		else if(table.height == 5) {mTables5.push_back(table.id);}
		else if(table.height == 10) {mTables10.push_back(table.id);}
		else if(table.height == 15) {mTables15.push_back(table.id);}
        typeCorrect=true;
        ROS_INFO_STREAM("[RN] registered table WS" << mTables.size() << " as T" << table.id);

      }
      if(!typeCorrect)
        ROS_ERROR_STREAM("[RN] Unknown table type \"" << table.type << "\" in worldmodel for table: " << table.id);
    }
    std::vector<WP> waypoints = wmc.getStrings("/Waypoint/*/ID");
    sort(waypoints.begin(), waypoints.end());
    for(const WP& wp : waypoints) {
      std::smatch results;
      if(!std::regex_search(wp, results, std::regex("[0-9]+")) || results.size()==0)
        ROS_ERROR_STREAM("[RN] Waypoint identifier invalid: " << wp);
        try{
          unsigned int wpID = boost::lexical_cast<unsigned int>(results[0]);
          if(wpID > maxTableId) {
            mWaypoints.push_back(wpID);
            ROS_INFO_STREAM("Waypoint WP" << mWaypoints.size() << " registered as " << wp);
          }
        }catch(const std::exception& e)  {
          ROS_ERROR_STREAM("[RN] Waypoint number invalid: " << results[0]);
        }
    }
	// generates random tasks depending on the discipline
	run tasks = auto_task_creation();
	for(size_t i=0; i< tasks.size(); ++i) {
		cout<<tasks.at(i).at(0)<<" "<<tasks.at(i).at(1)<<" "<<tasks.at(i).at(2)<<" "<<tasks.at(i).at(3)<<"\n";
	}
}

// reads the parameters
void ReceiverNode::readParameters()
{
	// DEFAULT WIEDER AUF -1
	ros::param::param<int>("/robOTTO/optimizer/testInstance", mdiscipline, 7);
	ros::param::param<bool>("~/referre", paramContainerInShelf, false);
	ros::param::param<bool>("~/referre", paramContainerOnTurntable, false);
	ros::param::param<bool>("~/referre", paramContainerOnPpt, false);
	ros::param::param<bool>("~/referre", paramFlexibleHeight, false);
	
	switch (mdiscipline) {
		case 0	: {
      ros::param::param<map<string, int>>("~/BNT", paramBNT, {{"tables" , 10}, {"waypoints" , 9}});
			break;
			}
		case 7	: {
			ros::param::param<map<string, int>>("~/Final", paramFinal, {{"objects" , 10}, {"tables" , 5},
				 {"decoys" , 8}, {"pick_shelf" , 2}, {"pick_turntables" , 1}, {"place_shelf" , 1},
				 {"place_turntables" , 0}, {"place_cavity_plattforms" , 3}, {"R_Container" , 1}, {"B_Container" , 1},
				 {"pick_tables0", 1}, {"pick_tables5", 1}, {"pick_tables10", 2}, {"pick_tables15", 1},
				 {"pick_cavity_plattforms", 0}});
			break;
			}
		default : throw 100;
		 
	}
	for (int i=robotto_msgs::ATWORK_START; i<robotto_msgs::ATWORK_END; ++i) {
		mObjects.push_back(i);
	}
	mPptObjects = mObjects;
	if(paramFinal["place_cavity_plattforms"] > 0 && mPptObjects.size() == 0) {throw 228;};
	for (int i=robotto_msgs::ROCKIN_START; i<robotto_msgs::ROCKIN_END; ++i) {
		mObjects.push_back(i);
	}
}

void ReceiverNode::readParameters(unsigned int objects, unsigned int tables, unsigned int decoys, unsigned int pickShelf,
                                  unsigned int pickTT, unsigned int placeShelf, unsigned int placeTT, unsigned int placePPT,
                                  unsigned int containerRed, unsigned int containerBlue, 
                                  bool containerInShelf, bool containerOnTT, bool containerOnPPT)
{
  paramContainerInShelf=containerInShelf;
  paramContainerOnTurntable=containerOnTT;
  paramContainerOnPpt=containerOnPPT;
  paramFinal={{"objects" , objects}, {"tables" , tables}, {"decoys" , decoys}, {"pick_shelf" , pickShelf}, {"pick_turntables" , pickTT}, 
             {"place_shelf" , placeShelf}, {"place_turntables" , placeTT}, {"place_cavity_plattforms" , placePPT}, 
             {"R_Container" , containerRed}, {"B_Container" , containerBlue}};
	for (int i=robotto_msgs::ATWORK_START; i<robotto_msgs::ATWORK_END; ++i) {
		mObjects.push_back(i);
	}
	mPptObjects = mObjects;
	if(paramFinal["place_cavity_plattforms"] > 0 && mPptObjects.size() == 0) {throw 228;};
	for (int i=robotto_msgs::ROCKIN_START; i<robotto_msgs::ROCKIN_END; ++i) {
		mObjects.push_back(i);
	}

}

ReceiverNode::ReceiverNode(const ArenaDescription& arena, const TaskDefinitions& tasks) {
    mTableMapping.resize(arena.workstations.size());
    unsigned int i=0;
    for(const auto& ws : arena.workstations) {
      if(ws.second == "PPT") {
        mPpts.push_back(i);
        continue;
      }
      if(ws.second == "SH") {
        mShelfs.push_back(i);
        continue;
      }
      if(ws.second == "CB" || ws.second == "RTT") {
        mConveyors.push_back(i);
        continue;
      }
      try{
        unsigned int height = boost::lexical_cast<unsigned int>(ws.second);
        switch(height) {
          case(0): mTables0.push_back(i); break;
          case(5): mTables5.push_back(i); break;
          case(10): mTables10.push_back(i); break;
          case(15): mTables15.push_back(i); break;
          default: ROS_ERROR_STREAM_NAMED("[REFBOX]", "Invalid height: " << height << " specified for normal workstation: " << ws.second); continue;
        }
      }
      catch(const std::exception& e) {
        ROS_ERROR_STREAM_NAMED("[REFBOX]", "Invalid workstation type specified: " << ws.second << " for workstation: " << ws.first);
        continue;
      }
      mTableMapping[i++]=ws.first;
    }
}
