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
#include <mouseWatcher.h>

#include "main.h"

///global data
PandaFramework framework;
WindowFramework *window;
CollideMask mask = BitMask32(0x10);
PT(RNNavMesh)navMesh;
PT(RNCrowdAgent)crowdAgent;
NodePath sceneNP;
bool setupCleanupFlag = true;
bool toggleDebugFlag = false;
bool halfVel = true;
int query = 0;
ValueList<LPoint3f> areaPointList;
vector<int> areaRefs;
ValueList<LPoint3f> linkPointPair;
vector<int> linkRefs;
bool firstSetup = true;

///functions' declarations
void changeSpeed(const Event*, void*);
void cycleQueries(const Event*, void*);
void addArea(const Event*, void*);
void removeArea(const Event*, void*);
void addLink(const Event*, void*);
void removeLink(const Event*, void*);
void enableDisableArea(const Event*, void* data);
void toggleDebugDraw(const Event*, void*);
void toggleSetupCleanup(const Event*, void*);
void placeCrowdAgent(const Event*, void*);
void setMoveTarget(const Event*, void*);
PT(CollisionEntry)getCollisionEntryFromCamera();

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
            "Press \"s\" to toggle setup/cleanup\n"
            "When nav mesh is not set up:\n"
            "\t- press \"a\" to add points of an area (convex volume)  under mouse cursor\n"
            " \t(\"shift-a\" for last point)\n"
            "\t- press \"r\" to remove area under mouse cursor\n\n"
            "When nav mesh is set up:\n"
            "\t- press \"d\" to toggle debug drawing\n"
            "\t- press \"p\" to place agent under mouse cursor\n"
            "\t- press \"t\" to set agent's target under mouse cursor\n"
            "\t- press \"v\" to change agent's speed\n"
            "\t- press \"q\" to cycle queries\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-0.2, 0.0, -0.4);
	textNodePath.set_scale(0.035);

	// create a nav mesh manager
	WPT(RNNavMeshManager)navMesMgr = new RNNavMeshManager(window->get_render(), mask);

	// get a sceneNP as owner model
	sceneNP = window->load_model(framework.get_models(), "dungeon.egg");
	sceneNP.set_collide_mask(mask);
	sceneNP.reparent_to(window->get_render());

	// create a nav mesh and attach it to render
	NodePath navMeshNP = navMesMgr->create_nav_mesh();
	navMesh = DCAST(RNNavMesh, navMeshNP.node());

	// mandatory: set sceneNP as owner of navMesh
	navMesh->set_owner_node_path(sceneNP);

	// reparent navMeshNP to a reference NodePath
	navMeshNP.reparent_to(window->get_render());

	// set nav mesh type
	navMesh->set_nav_mesh_type_enum(RNNavMesh::SOLO);
//	navMesh->set_nav_mesh_type_enum(RNNavMesh::TILE);
//	navMesh->set_nav_mesh_type_enum(RNNavMesh::OBSTACLE);

	// get the agent model
	NodePath agentNP = window->load_model(framework.get_models(), "eve.egg");
	agentNP.set_scale(0.40);

	// create the crowd agent and set the position
	NodePath crowdAgentNP = navMesMgr->create_crowd_agent("crowdAgent");
	crowdAgent = DCAST(RNCrowdAgent, crowdAgentNP.node());

	// attach the agent model to crowdAgent
	agentNP.reparent_to(crowdAgentNP);

	// start the path finding default update task
	navMesMgr->start_default_update();

	// toggle setup (true) and cleanup (false)
	framework.define_key("s", "toggleSetupCleanup", &toggleSetupCleanup,
			(void*) &setupCleanupFlag);

	// toggle debug draw
	framework.define_key("d", "toggleDebugDraw", &toggleDebugDraw,
			(void*) &toggleDebugFlag);

	// place crowd agent
	framework.define_key("p", "placeCrowdAgent", &placeCrowdAgent,
			nullptr);

	// handle move target on scene surface
	framework.define_key("t", "setMoveTarget", &setMoveTarget,
			nullptr);

	// add areas
	bool TRUE = true, FALSE = false;
	framework.define_key("a", "addArea", &addArea, (void*) &TRUE);
	framework.define_key("shift-a", "addAreaLast", &addArea, (void*) &FALSE);
	// remove areas
	framework.define_key("r", "removeArea", &removeArea, NULL);

	// add links
	framework.define_key("l", "addLink", &addLink, NULL);
	// remove links
	framework.define_key("k", "removeLink", &removeLink, NULL);

	// open/close area
	framework.define_key("o", "enableDisableArea", &enableDisableArea, NULL);

	// handle change speed
	framework.define_key("v", "changeSpeed", &changeSpeed, NULL);

	// handle cycle queries
	framework.define_key("q", "cycleQueries", &cycleQueries, NULL);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-10.0, 90.0, -12.0);
	trackball->set_hpr(0.0, 35.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}

///functions' definitions
// handle change speed
void changeSpeed(const Event* e, void* data)
{
	nassertv_always(crowdAgent)

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

// cycle over queries
void cycleQueries(const Event*, void*)
{
	nassertv_always(crowdAgent and navMesh)

	NodePath crowdAgentNP = NodePath::any_path(crowdAgent);
	switch (query)
	{
	case 0:
	{
		cout << "get path find to follow" << endl;
		ValueList<LPoint3f> areaPointList = navMesh->path_find_follow(
				crowdAgentNP.get_pos(), crowdAgent->get_move_target());
		for (int i = 0; i < areaPointList.size(); ++i)
		{
			cout << "\t" << areaPointList[i] << endl;
		}
	}
		break;
	case 1:
	{
		cout << "get path find to follow straight" << endl;
		ValueList<Pair<LPoint3f, unsigned char> > pointFlagList =
				navMesh->path_find_straight(crowdAgentNP.get_pos(),
						crowdAgent->get_move_target(),
						RNNavMesh::NONE_CROSSINGS);
		for (int i = 0; i < pointFlagList.size(); ++i)
		{
			string pathFlag;
			switch (pointFlagList[i].get_second())
			{
			case RNNavMesh::START:
				pathFlag = "START";
				break;
			case RNNavMesh::END:
				pathFlag = "END";
				break;
			case RNNavMesh::OFFMESH_CONNECTION:
				pathFlag = "OFFMESH_CONNECTION";
				break;
			default:
				break;
			}
			cout << "\t" << pointFlagList[i].get_first() << ", " << pathFlag
					<< endl;
		}
	}
		break;
	case 2:
	{
		cout << "check walkability" << endl;
		LPoint3f hitPoint = navMesh->ray_cast(crowdAgentNP.get_pos(),
				crowdAgent->get_move_target());
		if (hitPoint == crowdAgent->get_move_target())
		{
			cout << "\t" << "walkable!" << endl;
		}
		else
		{
			cout << "\t" << "not walkable!" << endl;
		}
	}
		break;
	case 3:
	{
		cout << "get distance to wall" << endl;
		float distance = navMesh->distance_to_wall(crowdAgentNP.get_pos());
		cout << "\t" << distance << endl;
	}
		break;
	default:
		break;
	}
	query += 1;
	query = query % 4;
}

// add area's (convex volume) points
void addArea(const Event*, void* data)
{
	nassertv_always(navMesh)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		bool addPoint = *reinterpret_cast<bool*>(data);
		LPoint3f point = entry0->get_surface_point(NodePath());
		if (addPoint)
		{
			RNNavMeshManager::get_global_ptr()->debug_draw_reset();
			// add to list
			areaPointList.add_value(point);
			RNNavMeshManager::get_global_ptr()->debug_draw_primitive(
					RNNavMeshManager::POINTS, areaPointList,
					LVecBase4f(1.0, 0.0, 0.0, 1.0), 4.0);
			cout << point << endl;
		}
		else
		{
			RNNavMeshManager::get_global_ptr()->debug_draw_reset();
			// add last point to list
			areaPointList.add_value(point);
			cout << point << endl;
			// add convex volume (area)
			int ref = navMesh->add_convex_volume(areaPointList,
					RNNavMesh::POLYAREA_DOOR);
			cout << "Added (temporary) area with ref: " << ref << endl;
			areaRefs.push_back(ref);
			// reset list
			areaPointList.clear();
		}
	}
}

// remove an area (convex volume)
void removeArea(const Event*, void* data)
{
	nassertv_always(navMesh)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		LPoint3f point = entry0->get_surface_point(NodePath());
		// try to remove area
		int ref = navMesh->remove_convex_volume(point);
		if (ref >= 0)
		{
			cout << "Removed area with ref: " << ref << endl;
		}
	}
}

// add a link's (off mesh connection) point pair
void addLink(const Event*, void* data)
{
	nassertv_always(navMesh)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		LPoint3f point = entry0->get_surface_point(NodePath());
		if (linkPointPair.get_num_values() == 0)
		{
			RNNavMeshManager::get_global_ptr()->debug_draw_reset();
			// add start point to list
			linkPointPair.add_value(point);
			RNNavMeshManager::get_global_ptr()->debug_draw_primitive(
					RNNavMeshManager::POINTS, linkPointPair,
					LVecBase4f(0.0, 0.0, 1.0, 1.0), 4.0);
			cout << point << endl;
		}
		else
		{
			RNNavMeshManager::get_global_ptr()->debug_draw_reset();
			// add end point to list
			linkPointPair.add_value(point);
			cout << point << endl;
			// add off mesh connection (link)
			int ref = navMesh->add_off_mesh_connection(linkPointPair, true);
			cout << "Added (temporary) bidirectional link with ref: " << ref << endl;
			linkRefs.push_back(ref);
			// reset list
			linkPointPair.clear();
		}
	}
}

// remove a link (off mesh connection)
void removeLink(const Event*, void* data)
{
	nassertv_always(navMesh)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		LPoint3f point = entry0->get_surface_point(NodePath());
		// try to remove link
		int ref = navMesh->remove_off_mesh_connection(point);
		if (ref >= 0)
		{
			cout << "Removed link with ref: " << ref << endl;
		}
	}
}

// enable disable area (convex volume)
void enableDisableArea(const Event*, void* data)
{
	nassertv_always(navMesh)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		LPoint3f point = entry0->get_surface_point(NodePath());
		// try to get area'settings by inside point
		RNConvexVolumeSettings settings = navMesh->get_convex_volume_settings(
				point);
		if (settings.get_ref() >= 0)
		{
			nassertv_always(
					navMesh->get_convex_volume_settings(settings.get_ref())
							== settings)
			// found a area: check if open or closed
			if (settings.get_flags() & RNNavMesh::POLYFLAGS_DISABLED)
			{
				// area is closed (convex volume disabled): open
				cout << "Open the area: " << endl;
			}
			else
			{
				// area is open (convex volume disabled): close
				cout << "Close the area: " << endl;
			}
			// switch area open/close
			settings.set_flags(
					settings.get_flags() ^ RNNavMesh::POLYFLAGS_DISABLED);
			// update settings
			navMesh->set_convex_volume_settings(settings.get_ref(), settings);
			cout << "\tref: " << settings.get_ref() << " | "
					"area: " << settings.get_area() << " | "
					"flags: " << settings.get_flags() << endl;
			// just for debug draw the agent's found path
			navMesh->path_find_follow(NodePath::any_path(crowdAgent).get_pos(),
					crowdAgent->get_move_target());
		}
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

// toggle debug draw
void toggleDebugDraw(const Event* e, void* data)
{
	bool* toggleDebugFlag = reinterpret_cast<bool*>(data);
	if (navMesh->toggle_debug_drawing(*toggleDebugFlag) >= 0)
	{
		*toggleDebugFlag = not *toggleDebugFlag;
	}
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
		if (firstSetup)
		{
			// first set initial position and target
			NodePath::any_path(crowdAgent).set_pos(LPoint3f(0.0, 15.0, 10.0));
			// then attach the crowd agent to the nav mesh
			navMesh->add_crowd_agent(NodePath::any_path(crowdAgent));
			crowdAgent->set_move_target(LPoint3f(0.0, 20.0, 10.0));
			firstSetup = false;
		}

		// show debug draw
		navMesh->toggle_debug_drawing(true);
		toggleDebugFlag = false;
		{
			// show areas
			vector<int>::iterator refI = areaRefs.begin();
			while (refI != areaRefs.end())
			{
				ValueList<LPoint3f> points = navMesh->get_convex_volume_by_ref(
						(*refI));
				if (points.get_num_values() == 0)
				{
					cout << "Area's invalid ref: " << (*refI) << " ...removing"
							<< endl;
					areaRefs.erase(refI);
					continue;
				}
				LPoint3f centroid = LPoint3f::zero();
				for (int p = 0; p < points.size(); ++p)
				{
					centroid += points[p];
				}
				centroid /= points.get_num_values();
				RNConvexVolumeSettings settings =
						navMesh->get_convex_volume_settings(centroid);

				nassertv_always(
						settings
								== navMesh->get_convex_volume_settings((*refI)));

				cout << "Area n. " << (refI - areaRefs.begin()) << endl;
				cout << "\tref: " << settings.get_ref() << " | "
						"area: " << settings.get_area() << " | "
						"flags: " << settings.get_flags() << endl;
				//
				++refI;
			}
		}
		{
//			// show links
//			vector<int>::iterator refI = areaRefs.begin();
//			while (refI != areaRefs.end())
//			{
//				ValueList<LPoint3f> points = navMesh->get_convex_volume_by_ref(
//						(*refI));
//				if (points.get_num_values() == 0)
//				{
//					cout << "Area's invalid ref: " << (*refI) << " ...removing"
//							<< endl;
//					areaRefs.erase(refI);
//					continue;
//				}
//				LPoint3f centroid = LPoint3f::zero();
//				for (int p = 0; p < points.size(); ++p)
//				{
//					centroid += points[p];
//				}
//				centroid /= points.get_num_values();
//				RNConvexVolumeSettings settings =
//						navMesh->get_convex_volume_settings(centroid);
//
//				nassertv_always(
//						settings
//								== navMesh->get_convex_volume_settings((*refI)));
//
//				cout << "Area n. " << (refI - areaRefs.begin()) << endl;
//				cout << "\tref: " << settings.get_ref() << " | "
//						"area: " << settings.get_area() << " | "
//						"flags: " << settings.get_flags() << endl;
//				//
//				++refI;
//			}
		}
	}
	else
	{
		// false: cleanup
		navMesh->cleanup();
		areaPointList.clear();
		linkPointPair.clear();
		// now crowd agents and obstacles are detached:
		// prevent to make them disappear from the scene
		for (int i = 0; i < navMesh->get_num_crowd_agents(); ++i)
		{
			NodePath::any_path(navMesh->get_crowd_agent(i)).reparent_to(
					window->get_render());
		}
	}
	*setupCleanupFlag = not *setupCleanupFlag;
}

// place crowd agent
void placeCrowdAgent(const Event* e, void* data)
{
	nassertv_always(navMesh and crowdAgent)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		// remove agent from nav mesh
		navMesh->remove_crowd_agent(NodePath::any_path(crowdAgent));
		LPoint3f point = entry0->get_surface_point(NodePath());
		NodePath::any_path(crowdAgent).set_pos(point);
		// re-add agent to nav mesh
		navMesh->add_crowd_agent(NodePath::any_path(crowdAgent));
		// just for debug draw the agent's found path
		navMesh->path_find_follow(point, crowdAgent->get_move_target());
	}
}

// handle set move target
void setMoveTarget(const Event* e, void* data)
{
	nassertv_always(navMesh and crowdAgent)

	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		LPoint3f target = entry0->get_surface_point(NodePath());
		crowdAgent->set_move_target(target);
		// just for debug draw the agent's found path
		navMesh->path_find_follow(
				NodePath::any_path(crowdAgent).get_pos(), target);
	}
}
