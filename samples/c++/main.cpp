/**
 * \file main.cpp
 *
 * \date 2016-03-30
 * \author consultit
 */

#include <pandaFramework.h>
#include <auto_bind.h>
#include <load_prc_file.h>
#include <rnNavMeshManager.h>
#include <rnNavMesh.h>
#include <rnCrowdAgent.h>
#include <collisionRay.h>
#include <mouseWatcher.h>
#include <random>
#include <bamFile.h>

#include "main.h"

///functions' declarations
void loadAllScene();
void restoreAllScene();
void getOwnerModel();
void getAgentModelAnims();
bool readFromBamFile(string);
void writeToBamFileAndExit(const Event*, void*);
void printCreationParameters();
void setParametersBeforeCreation();
void toggleDebugDraw(const Event*, void*);
void toggleSetupCleanup(const Event*, void*);
void handleCrowdAgentEvent(const Event*, void*);
void placeCrowdAgents(const Event*, void*);
void setMoveTarget(const Event*, void*);
void handleObstacles(const Event*, void*);
AsyncTask* updateTask;
AsyncTask::DoneStatus updateNavMesh(GenericAsyncTask*, void*);
LPoint3f getRandomPos(NodePath);
PT(CollisionEntry)getCollisionEntryFromCamera();

///global data
PandaFramework framework;
WindowFramework *window;
CollideMask mask = BitMask32(0x10);
PT(RNNavMesh)navMesh;
const int NUMAGENTS = 2;//XXX
PT(RNCrowdAgent)crowdAgent[2];
//models and animations
NodePath sceneNP, agentNP[2];
string sceneFile("nav_test.egg");
string agentFile[2] =
{ "eve.egg", "ralph.egg" };
string agentAnimFiles[2][2] =
{
{ "eve-walk.egg", "eve-offbalance.egg" },
{ "ralph-walk.egg", "ralph-offbalance.egg" } };
const float rateFactor = 1.50;
PT(AnimControl)agentAnimCtls[2][2];
//obstacle model
string obstacleFile("plants2.egg");
//bame file
string bamFileName("nav_mesh.boo");
//support
random_device rd;

int main(int argc, char *argv[])
{
	// Load your application's configuration
	load_prc_file_data("", "model-path " + dataDir);
	load_prc_file_data("", "win-size 1024 768");
	load_prc_file_data("", "show-frame-rate-meter #t");
	load_prc_file_data("", "sync-video #t");
	// Setup your application
	framework.open_framework(argc, argv);
	framework.set_window_title("p3recastnavigation");
	window = framework.open_window();
	if (window != (WindowFramework *) NULL)
	{
		cout << "Opened the window successfully!\n";
		window->enable_keyboard();
		window->setup_trackball();
	}

	/// here is room for your own code

	/// typed object init; not needed if you build inside panda source tree
	RNNavMesh::init_type();
	RNCrowdAgent::init_type();
	RNNavMeshManager::init_type();
	RNNavMesh::register_with_read_factory();
	RNCrowdAgent::register_with_read_factory();
	///
	// print some help to screen
	PT(TextNode) text;
	text = new TextNode("Help");
	text->set_text(
			"- press \"d\" to toggle debug drawing\n"
			"- press \"s\" to toggle setup cleanup\n"
			"- press \"p\" to place agents randomly\n"
			"- press \"t\" to set agents' target under mouse cursor\n"
			"- press \"o\" to add obstacle under mouse cursor\n"
			"- press \"shift-o\" to remove obstacle under mouse cursor\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-1.25, 0.0, 0.9);
	textNodePath.set_scale(0.035);

	// create a nav mesh manager
	WPT(RNNavMeshManager)navMesMgr = new RNNavMeshManager(window->get_render(), mask);

	// print creation parameters: defult values
	cout << endl << "Default creation parameters:";
	printCreationParameters();

	// set creation parameters as strings before nav meshes/crowd agent creation
	cout << endl << "Current creation parameters:";
	setParametersBeforeCreation();

	// load or restore all scene stuff: if passed an argument
	// try to read it from bam file
	if ((not (argc > 1)) or (not readFromBamFile(argv[1])))
	{
		// no argument or no valid bamFile
		loadAllScene();
	}
	else
	{
		// valid bamFile
		restoreAllScene();
	}

	/// first option: start the path finding default update task
///	navMesMgr->start_default_update();

/// second option: start a path finding custom update task
	updateTask = new GenericAsyncTask("updateNavMesh", &updateNavMesh,
			(void*) navMesh.p());
	framework.get_task_mgr().add(updateTask);

	// setup  debug drawing 
	navMesh->enable_debug_drawing(window->get_camera_group());

	/// set events' callbacks
	// toggle debug draw
	bool toggleDebugFlag = false;
	framework.define_key("d", "toggleDebugDraw", &toggleDebugDraw,
			(void*) &toggleDebugFlag);

	// toggle setup (true) and cleanup (false)
	bool setupCleanupFlag = false;
	framework.define_key("s", "toggleSetupCleanup", &toggleSetupCleanup,
			(void*) &setupCleanupFlag);

	// handle CrowdAgents' events
	framework.define_key("move-event", "handleCrowdAgentEvent",
			&handleCrowdAgentEvent, nullptr);

	// place crowd agents randomly
	framework.define_key("p", "placeCrowdAgents", &placeCrowdAgents,
			nullptr);

	// handle move target on scene surface
	framework.define_key("t", "setMoveTarget", &setMoveTarget, nullptr);

	// handle obstacle addition
	bool TRUE = true;
	framework.define_key("o", "addObstacle", &handleObstacles, (void*) &TRUE);

	// handle obstacle removal
	bool FALSE = false;
	framework.define_key("shift-o", "removeObstacle", &handleObstacles,
			(void*) &FALSE);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-10.0, 90.0, -2.0);
	trackball->set_hpr(0.0, 15.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}

///functions' definitions
// load all scene stuff
void loadAllScene()
{
	RNNavMeshManager* navMesMgr = RNNavMeshManager::get_global_ptr();
	// get a sceneNP as owner model
	getOwnerModel();

	// create a nav mesh and attach it to render
	NodePath navMeshNP = navMesMgr->create_nav_mesh();
	navMesh = DCAST(RNNavMesh, navMeshNP.node());

	// mandatory: set sceneNP as owner of navMesh
	navMesh->set_owner_node_path(sceneNP);

	// setup the nav mesh with scene as its owner object
	navMesh->setup();

	// reparent navMeshNP to a reference NodePath
	navMeshNP.reparent_to(window->get_render());

	// get agentNP[] (and agentAnimNP[]) as models for crowd agents
	getAgentModelAnims();

	// create crowd agents and attach agentNP[] (and agentAnimNP[]) as children
	for (int i = 0; i < NUMAGENTS; ++i)
	{
		// set parameter for crowd agent's type (RECAST or RECAST_KINEMATIC)
		string agentType;
		(i % 2) == 0 ? agentType = "recast" : agentType = "kinematic";
		navMesMgr->set_parameter_value(RNNavMeshManager::CROWDAGENT,
				"mov_type", agentType);
		// create the crowd agent
		NodePath crowdAgentNP = navMesMgr->create_crowd_agent(
				"crowdAgent" + str(i));
		crowdAgent[i] = DCAST(RNCrowdAgent, crowdAgentNP.node());
		// set the position randomly
		LPoint3f randPos = getRandomPos(sceneNP);
		crowdAgentNP.set_pos(randPos);
		// attach some geometry (a model) to crowdAgent
		agentNP[i].reparent_to(crowdAgentNP);
		// attach the crowd agent to the sceneNP's nav mesh
		navMesh->add_crowd_agent(crowdAgentNP);
	}
}

// restore all scene stuff
void restoreAllScene()
{
	// restore nav mesh
	NodePath navMeshNP = RNNavMeshManager::get_global_ptr()->get_nav_mesh(
			0);
	navMeshNP.reparent_to(window->get_render());
	navMesh = DCAST(RNNavMesh, navMeshNP.node());
	sceneNP = navMesh->get_owner_node_path();

	// restore crowd agents
	for (int i = 0; i < NUMAGENTS; ++i)
	{
		// create the crowd agent
		NodePath crowdAgentNP =
				RNNavMeshManager::get_global_ptr()->get_crowd_agent(i);
		crowdAgent[i] = DCAST(RNCrowdAgent, crowdAgentNP.node());
		// restore animations
		AnimControlCollection tmpAnims;
		auto_bind(crowdAgent[i], tmpAnims);
		for (int j = 0; j < tmpAnims.get_num_anims(); ++j)
		{
			agentAnimCtls[i][j] = tmpAnims.get_anim(j);
		}
	}
}

// load the owner model
void getOwnerModel()
{
	// get a model to use as nav mesh' owner object
	sceneNP = window->load_model(framework.get_models(), sceneFile);
	sceneNP.set_collide_mask(mask);
//	sceneNP.set_pos(5.0, 20.0, 5.0);
//	sceneNP.set_h(30.0);
//	sceneNP.set_scale(2.0);
}

// load the agents' models and anims
void getAgentModelAnims()
{
	// get some models, with animations, to attach to crowd agents
	for (int i = 0; i < NUMAGENTS; ++i)
	{
		// get the model
		agentNP[i] = window->load_model(framework.get_models(), agentFile[i]);
		// set random scale (0.35 - 0.45)
		float scale = 0.35 + 0.1 * ((float) rd() / (float) rd.max());
		agentNP[i].set_scale(scale);
		// associate an anim with a given anim control
		AnimControlCollection tmpAnims;
		NodePath agentAnimNP[2];
		// first anim -> modelAnimCtls[i][0]
		agentAnimNP[0] = window->load_model(agentNP[i],
				agentAnimFiles[i][0]);
		auto_bind(agentNP[i].node(), tmpAnims);
		agentAnimCtls[i][0] = tmpAnims.get_anim(0);
		tmpAnims.clear_anims();
		agentAnimNP[0].detach_node();
		// second anim -> modelAnimCtls[i][1]
		agentAnimNP[1] = window->load_model(agentNP[i],
				agentAnimFiles[i][1]);
		auto_bind(agentNP[i].node(), tmpAnims);
		agentAnimCtls[i][1] = tmpAnims.get_anim(0);
		tmpAnims.clear_anims();
		agentAnimNP[1].detach_node();
		// reparent all node paths
		agentAnimNP[0].reparent_to(agentNP[i]);
		agentAnimNP[1].reparent_to(agentNP[i]);
	}
}

// read nav mesh from a file
bool readFromBamFile(string fileName)
{
	return RNNavMeshManager::get_global_ptr()->read_from_bam_file(fileName);
}

// write nav mesh to a file
void writeToBamFileAndExit(const Event* e, void* data)
{
	string fileName = *reinterpret_cast<string*>(data);
	RNNavMeshManager::get_global_ptr()->write_to_bam_file(fileName);

	/// second option: remove custom update updateTask
	framework.get_task_mgr().remove(updateTask);
	// delete nav mesh manager
	delete RNNavMeshManager::get_global_ptr();
	// close the window framework
	framework.close_framework();
	//
	exit(0);
}

// print creation parameters
void printCreationParameters()
{
	RNNavMeshManager* navMesMgr = RNNavMeshManager::get_global_ptr();
	//
	ValueListString valueList = navMesMgr->get_parameter_name_list(
			RNNavMeshManager::NAVMESH);
	cout << endl << "RNNavMesh creation parameters:" << endl;
	for (int i = 0; i < valueList.get_num_values(); ++i)
	{
		cout << "\t" << valueList[i] << " = "
				<< navMesMgr->get_parameter_value(RNNavMeshManager::NAVMESH,
						valueList[i]) << endl;
	}
	//
	valueList = navMesMgr->get_parameter_name_list(
			RNNavMeshManager::CROWDAGENT);
	cout << endl << "RNCrowdAgent creation parameters:" << endl;
	for (int i = 0; i < valueList.get_num_values(); ++i)
	{
		cout << "\t" << valueList[i] << " = "
				<< navMesMgr->get_parameter_value(RNNavMeshManager::CROWDAGENT,
						valueList[i]) << endl;
	}
}

// set parameters as strings before nav meshes/crowd agents creation
void setParametersBeforeCreation()
{
	RNNavMeshManager* navMesMgr = RNNavMeshManager::get_global_ptr();
	// tweak some nav mesh parameter
	navMesMgr->set_parameter_value(RNNavMeshManager::NAVMESH, "navmesh_type",
			"obstacle");
	navMesMgr->set_parameter_value(RNNavMeshManager::NAVMESH, "build_all_tiles",
			"true");
	navMesMgr->set_parameter_value(RNNavMeshManager::NAVMESH, "agent_max_climb",
			"2.5");

	ValueListString valueList;
	// set some off mesh connections: "area_type@flag1[:flag2...:flagN]@cost"
	valueList.add_value("31.6,24.5,-2.0:20.2,9.4,-2.4@true");
	valueList.add_value("21.1,-4.5,-2.4:32.3,-3.0,-1.5@true");
	valueList.add_value("1.2,-13.1,15.2:11.8,-18.3,10.0@true");
	navMesMgr->set_parameter_values(RNNavMeshManager::NAVMESH,
			"offmesh_connection", valueList);
	// set some convex volumes: "x1,y1,z1[:x2,y2,z2...:xN,yN,zN]@area_type"
	valueList.clear();
	valueList.add_value(
			"-15.2,-22.9,-2.4:-13.4,-22.6,-2.4:-13.1,-26.5,-2.4:-16.4,-26.4,-2.7@1");
	navMesMgr->set_parameter_values(RNNavMeshManager::NAVMESH, "convex_volume",
			valueList);

	// set crowd agent throwing events
	valueList.clear();
	valueList.add_value("move@move-event@0.5");
	navMesMgr->set_parameter_values(RNNavMeshManager::CROWDAGENT,
			"thrown_events", valueList);
	//
	printCreationParameters();
}

// toggle debug draw
void toggleDebugDraw(const Event* e, void* data)
{
	bool* toggleDebugFlag = reinterpret_cast<bool*>(data);
	*toggleDebugFlag = not *toggleDebugFlag;
	navMesh->toggle_debug_drawing(*toggleDebugFlag);
}

// toggle setup/cleanup
void toggleSetupCleanup(const Event* e, void* data)
{
	bool* setupCleanupFlag = reinterpret_cast<bool*>(data);
	if (*setupCleanupFlag)
	{
		// true: setup
		navMesh->set_owner_node_path(sceneNP);
		navMesh->setup();
		navMesh->enable_debug_drawing(window->get_camera_group());
		//
		updateTask = new GenericAsyncTask("updateNavMesh", &updateNavMesh,
				(void*) navMesh.p());
		framework.get_task_mgr().add(updateTask);
	}
	else
	{
		framework.get_task_mgr().remove(updateTask);
		// false: cleanup
		navMesh->cleanup();
	}
	*setupCleanupFlag = not *setupCleanupFlag;
}

// handle crowd agent's events
void handleCrowdAgentEvent(const Event* e, void* data)
{
	PT(RNCrowdAgent)crowAgent = DCAST(RNCrowdAgent, e->get_parameter(0).get_ptr());
	NodePath agent = NodePath::any_path(crowAgent);
	cout << "move-event - " << agent.get_name() << " - "<< agent.get_pos() << endl;
}

// place crowd agents randomly
void placeCrowdAgents(const Event* e, void* data)
{
	for (int i = 0; i < NUMAGENTS; ++i)
	{
		// remove agent from nav mesh
		navMesh->remove_crowd_agent(NodePath::any_path(crowdAgent[i]));
		// set its random position
		LPoint3f randPos = getRandomPos(sceneNP);
		NodePath::any_path(crowdAgent[i]).set_pos(randPos);
		// re-add agent to nav mesh
		navMesh->add_crowd_agent(NodePath::any_path(crowdAgent[i]));
	}
}

// throws a ray and returns the first collision entry or nullptr
PT(CollisionEntry)getCollisionEntryFromCamera()
{
	// get nav mesh manager
	RNNavMeshManager* navMeshMgr = RNNavMeshManager::get_global_ptr();
	// get the mouse watcher
	PT(MouseWatcher)mwatcher = DCAST(MouseWatcher, window->get_mouse().node());
	if (mwatcher->has_mouse())
	{
		// Get to and from pos in camera coordinates
		LPoint2f pMouse = mwatcher->get_mouse();
		//
		LPoint3f pFrom, pTo;
		NodePath mCamera = window->get_camera_group();
		PT(Lens)mCamLens = DCAST(Camera, mCamera.get_child(0).node())->get_lens();
		if (mCamLens->extrude(pMouse, pFrom, pTo))
		{
			// Transform to global coordinates
			pFrom = window->get_render().get_relative_point(mCamera,
					pFrom);
			pTo = window->get_render().get_relative_point(mCamera, pTo);
			LVector3f direction = (pTo - pFrom).normalized();
			navMeshMgr->get_collision_ray()->set_origin(pFrom);
			navMeshMgr->get_collision_ray()->set_direction(direction);
			navMeshMgr->get_collision_traverser()->traverse(window->get_render());
			// check collisions
			if (navMeshMgr->get_collision_handler()->get_num_entries() > 0)
			{
				// Get the closest entry
				navMeshMgr->get_collision_handler()->sort_entries();
				return navMeshMgr->get_collision_handler()->get_entry(0);
			}
		}
	}
	return nullptr;
}

// handle set move target
void setMoveTarget(const Event* e, void* data)
{
	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		LPoint3f target = entry0->get_surface_point(NodePath());
		for (int i = 0; i < navMesh->get_num_crowd_agents(); ++i)
		{
			(*navMesh)[i]->set_move_target(target);
		}
	}
}

// handle add/remove obstacles
void handleObstacles(const Event* e, void* data)
{
	bool addObstacle = *reinterpret_cast<bool*>(data);
	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		// get the hit object
		NodePath hitObject = entry0->get_into_node_path();
		// check if hitObject is the nav mesh owner object or
		// this last one is one of its anchestors
		if (addObstacle
				and ((hitObject == navMesh->get_owner_node_path())
						or (navMesh->get_owner_node_path().is_ancestor_of(
								hitObject))))
		{
			// the hit object is the scene
			// add an obstacle to the scene

			// get a model as obstacle
			NodePath obstacleNP = window->load_model(sceneNP, obstacleFile);
			obstacleNP.set_collide_mask(mask);
			// set random scale (0.01 - 0.02)
			float scale = 0.01 + 0.01 * ((float) rd() / (float) rd.max());
			obstacleNP.set_scale(scale);
			// set obstacle position
			LPoint3f pos = entry0->get_surface_point(sceneNP);
			obstacleNP.set_pos(pos);
			// try to add to nav mesh
			if (navMesh->add_obstacle(obstacleNP) < 0)
			{
				// something went wrong remove from scene
				obstacleNP.remove_node();
			}
		}
		// check if hitObject is not the nav mesh owner object and
		// this last one is not one of its anchestors
		else if ((not addObstacle)
				and ((hitObject != navMesh->get_owner_node_path())
						and (not navMesh->get_owner_node_path().is_ancestor_of(
								hitObject))))
		{
			// cycle the obstacle list
			for (int index = 0; index < navMesh->get_num_obstacles();
					++index)
			{
				unsigned int ref = navMesh->get_obstacle(index);
				NodePath obstacleNP = navMesh->get_obstacle_by_ref(ref);
				// check if the hitObject == obstacle or
				// obstacle is an ancestor of the hitObject
				if ((hitObject == obstacleNP)
						or (obstacleNP.is_ancestor_of(hitObject)))
				{
					// try to remove from nav mesh
					if (navMesh->remove_obstacle(obstacleNP)
							>= 0)
					{
						// all ok remove from scene
						obstacleNP.remove_node();
						hitObject.remove_node();
						break;
					}
				}
			}
		}
	}
}

// custom path finding update task to correct panda's Z to stay on floor
AsyncTask::DoneStatus updateNavMesh(GenericAsyncTask* task, void* data)
{
	PT(RNNavMesh)navMesh = reinterpret_cast<RNNavMesh*>(data);
	// call update for navMesh
	double dt = ClockObject::get_global_clock()->get_dt();
	navMesh->update(dt);
	// handle crowd agents' animation
	for (int i = 0; i < NUMAGENTS; ++i)
	{
		// get current velocity size
		float currentVelSize = crowdAgent[i]->get_actual_velocity().length();
		if (currentVelSize > 0.0)
		{
			// walk
			agentAnimCtls[i][0]->set_play_rate(currentVelSize / rateFactor);
			if (not agentAnimCtls[i][0]->is_playing())
			{
				agentAnimCtls[i][0]->loop(true);
			}
		}
		else
		{
			// check if crowd agent is on off mesh connection
			if (crowdAgent[i]->get_traversing_state() == RNCrowdAgent::STATE_OFFMESH)
			{
				// off-balance
				if (not agentAnimCtls[i][1]->is_playing())
				{
					agentAnimCtls[i][1]->loop(true);
				}
			}
			else
			{
				// stop any animation
				agentAnimCtls[i][0]->stop();
				agentAnimCtls[i][1]->stop();
			}
		}
	}
	//
	return AsyncTask::DS_cont;
}

// return a random point on the facing upwards surface of the model
LPoint3f getRandomPos(NodePath modelNP)
{
	// collisions are made wrt render
	RNNavMeshManager* navMeshMgr = RNNavMeshManager::get_global_ptr();
	// get the bounding box of scene
	LVecBase3f modelDims;
	LVector3f modelDeltaCenter;
	// modelRadius not used
	navMeshMgr->get_bounding_dimensions(modelNP, modelDims, modelDeltaCenter);
	// throw a ray downward from a point with z = double scene's height
	// and x,y randomly within the scene's (x,y) plane
	float x, y = 0.0;
	PairBoolFloat gotCollisionZ;
	// set the ray origin at double of maximum height of the model
	float zOrig = ((-modelDeltaCenter.get_z() + modelDims.get_z() / 2.0)
			+ modelNP.get_z()) * 2.0;
	do
	{
		x = modelDims.get_x() * ((float) rd() / (float) rd.max() - 0.5)
				- modelDeltaCenter.get_x() + modelNP.get_x();
		y = modelDims.get_y() * ((float) rd() / (float) rd.max() - 0.5)
				- modelDeltaCenter.get_y() + modelNP.get_y();
		gotCollisionZ = navMeshMgr->get_collision_height(LPoint3f(x, y, zOrig));

	} while (not gotCollisionZ.get_first());
	return LPoint3f(x, y, gotCollisionZ.get_second());
}
