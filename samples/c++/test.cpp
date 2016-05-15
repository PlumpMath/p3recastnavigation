/**
 * \file main.cpp
 *
 * \date 2016-03-30
 * \author consultit
 */

#include <pandaFramework.h>
#include <load_prc_file.h>
#include <rnNavMeshManager.h>
#include <rnNavMesh.h>
#include <rnCrowdAgent.h>

#include "main.h"

///functions' declarations
void changeSpeed(const Event*, void*);

///global data
PandaFramework framework;
WindowFramework *window;
PT(RNCrowdAgent)crowdAgent;
bool halfVel = true;

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

	cout << "create a nav mesh manager" << endl;
	WPT(RNNavMeshManager)navMesMgr = new RNNavMeshManager(window->get_render());

	cout << "get a sceneNP as owner model" << endl;
	NodePath sceneNP = window->load_model(framework.get_models(),
			"nav_test.egg");

	cout << "create a nav mesh and attach it to render" << endl;
	NodePath navMeshNP = navMesMgr->create_nav_mesh();
	PT(RNNavMesh)navMesh = DCAST(RNNavMesh, navMeshNP.node());

	cout << "mandatory: set sceneNP as owner of navMesh" << endl;
	navMesh->set_owner_node_path(sceneNP);

	cout << "setup the nav mesh with scene as its owner object" << endl;
	navMesh->setup();

	cout << "reparent navMeshNP to a reference NodePath" << endl;
	navMeshNP.reparent_to(window->get_render());

	cout << "get the agent model" << endl;
	NodePath agentNP = window->load_model(framework.get_models(), "eve.egg");
	agentNP.set_scale(0.40);

	cout << "create the crowd agent and set the position" << endl;
	NodePath crowdAgentNP = navMesMgr->create_crowd_agent("crowdAgent");
	crowdAgent = DCAST(RNCrowdAgent, crowdAgentNP.node());
	crowdAgentNP.set_pos(24.0, -20.4, -2.37);

	cout << "attach the agent model to crowdAgent" << endl;
	agentNP.reparent_to(crowdAgentNP);

	cout << "attach the crowd agent to the nav mesh" << endl;
	navMesh->add_crowd_agent(crowdAgentNP);

	cout << "start the path finding default update task" << endl;
	navMesMgr->start_default_update();

	cout << "enable debug draw" << endl;
	navMesh->enable_debug_drawing(window->get_camera_group());

	cout << "toggle debug draw" << endl;
	navMesh->toggle_debug_drawing(true);

	cout << "set crowd agent move target on scene surface" << endl;
	crowdAgent->set_move_target(LPoint3f(-20.5, 5.2, -2.36));

	ValueListLPoint3f pointList;
	cout << "get path find follow/straight/sliced" << endl;
//	pointList = navMesh->get_path_find_follow(crowdAgentNP.get_pos(),
//			crowdAgent->get_move_target());
	pointList = navMesh->get_path_find_straight(crowdAgentNP.get_pos(),
			crowdAgent->get_move_target(), RNNavMesh::NONE_CROSSINGS);
//	navMesh->get_path_find_sliced(crowdAgentNP.get_pos(),
//			crowdAgent->get_move_target());
	for (int i = 0; i < pointList.size(); ++i)
	{
		cout << pointList[i] << endl;
	}

// handle change speed
	framework.define_key("s", "changeSpeed", &changeSpeed, NULL);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-10.0, 90.0, -2.0);
	trackball->set_hpr(0.0, 15.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}

///functions' definitions
// handle change speed
void changeSpeed(const Event* e, void* data)
{
	RNCrowdAgentParams ap = crowdAgent->get_params();
	float vel = ap.get_maxSpeed();
	if (halfVel)
	{
		ap.set_maxSpeed(vel / 2.0);
	}
	else
	{
		ap.set_maxSpeed(vel * 2.0);
	}
	crowdAgent->set_params(ap);
	halfVel = not halfVel;
}
