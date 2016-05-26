'''
Created on Mar 24, 2016

@author: consultit
'''

# from direct.actor.Actor import Actor
import panda3d.core
from p3recastnavigation import RNNavMeshManager, RNNavMesh, ValueList_LPoint3f
from panda3d.core import load_prc_file_data, LPoint3f, BitMask32, NodePath, \
                LVecBase4f
from direct.showbase.ShowBase import ShowBase

dataDir = "../data"

mask = BitMask32(0x10)
navMesh = None
crowdAgent = None
sceneNP = None
setupCleanupFlag = True
toggleDebugFlag = False
halfVel = True
query = 0
pointList = ValueList_LPoint3f()
doorRefs = []
firstSetup = True

def changeSpeed():
    global crowdAgent, halfVel
    if crowdAgent == None:
        return
    
    ap = crowdAgent.get_params()
    vel = ap.get_maxSpeed()
    if halfVel:
        ap.set_maxSpeed(vel / 2.0)
    else:
        ap.set_maxSpeed(vel * 2.0)
    crowdAgent.set_params(ap)
    halfVel = not halfVel

def cycleQueries():
    global query, navMesh, crowdAgent
    if (crowdAgent == None) or (navMesh == None):
        return
    
    if query == 0:
        print("get path find to follow")
        pointList = navMesh.get_path_find_follow(
                crowdAgentNP.get_pos(), crowdAgent.get_move_target());
        for p in pointList:
            print("\t" + str(p))
    elif query == 1: 
        print("get path find to follow straight")
        pointFlagList = navMesh.get_path_find_straight(crowdAgentNP.get_pos(),
                        crowdAgent.get_move_target(), RNNavMesh.NONE_CROSSINGS);
        for pF in pointFlagList:
            pathFlag = None
            flag = pF.get_second()
            if flag == RNNavMesh.START:
                pathFlag = "START"
            elif flag == RNNavMesh.END:
                pathFlag = "END";
            elif flag == RNNavMesh.OFFMESH_CONNECTION:
                pathFlag = "OFFMESH_CONNECTION";
            print("\t" + str(pF.get_first()) + ", " + str(pathFlag))
    elif query == 2:
        print("check walkability")
        hitPoint = navMesh.check_walkability(
                crowdAgentNP.get_pos(), crowdAgent.get_move_target())
        if hitPoint == crowdAgent.get_move_target():
            print("\t" + "walkable!")
        else:
            print("\t" + "not walkable!")
    elif query == 3:
        print("get distance to wall")
        distance = navMesh.get_distance_to_wall(crowdAgentNP.get_pos())
        print("\t" + str(distance))    
    else:
        pass
    query += 1
    query = query % 4

def addDoor(data):
    global pointList, doorRefs, navMesh
    if navMesh == None:
        return
    
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        addPoint = data
        point = entry0.get_surface_point(NodePath())
        if addPoint:
            RNNavMeshManager.get_global_ptr().debug_draw_reset()
            # add to list
            pointList.add_value(point)
            RNNavMeshManager.get_global_ptr().debug_draw_primitive(
                    RNNavMeshManager.POINTS, pointList,
                    LVecBase4f(1.0, 0.0, 0.0, 1.0), 4.0)
            print(point)
        else:
            RNNavMeshManager.get_global_ptr().debug_draw_reset()
            # add last point to list
            pointList.add_value(point)
            print(point)
            # add convex volume (door)
            ref = navMesh.add_convex_volume(pointList, RNNavMesh.POLYAREA_DOOR)
            print("Added (temporary) door with ref: " + str(ref))
            doorRefs.append(ref)
            # reset list
            pointList.clear()

def removeDoor():
    global navMesh
    if navMesh == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to remove door
        ref = navMesh.remove_convex_volume(point)
        if ref >= 0:
            print("Removed door with ref: " + str(ref))

def openCloseDoor():
    global navMesh
    if navMesh == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to get door'settings by inside point
        settings = navMesh.get_convex_volume_settings(point)
        if settings.get_ref() >= 0:
            if not (navMesh.get_convex_volume_settings(settings.get_ref()) == settings):
                return
            
            # found a door: check if open or closed
            if settings.get_flags() & RNNavMesh.POLYFLAGS_DISABLED:
                # door is closed (convex volume disabled): open
                print("Open the door: ")
            else:
                # door is open (convex volume disabled): close
                print("Close the door: ")
            # switch door open/close
            settings.set_flags(settings.get_flags() ^ RNNavMesh.POLYFLAGS_DISABLED)
            # update settings
            navMesh.set_convex_volume_settings(settings.get_ref(), settings)
            print("\tref: " + str(settings.get_ref()) + " | "
                    "area: " + str(settings.get_area()) + " | "
                    "flags: " + str(settings.get_flags()))   

# throws a ray and returns the first collision entry or nullptr
def getCollisionEntryFromCamera():
    global app
    # get nav mesh manager
    navMeshMgr = RNNavMeshManager.get_global_ptr()
    # get the mouse watcher
    mwatcher = app.mouseWatcherNode
    if mwatcher.has_mouse():
        # Get to and from pos in camera coordinates
        pMouse = mwatcher.get_mouse()
        #
        pFrom, pTo = (LPoint3f(), LPoint3f())
        if app.camLens.extrude(pMouse, pFrom, pTo):
            # Transform to global coordinates
            pFrom = app.render.get_relative_point(app.cam, pFrom)
            pTo = app.render.get_relative_point(app.cam, pTo)
            direction = (pTo - pFrom).normalized()
            navMeshMgr.get_collision_ray().set_origin(pFrom)
            navMeshMgr.get_collision_ray().set_direction(direction)
            navMeshMgr.get_collision_traverser().traverse(app.render)
            # check collisions
            if navMeshMgr.get_collision_handler().get_num_entries() > 0:
                # Get the closest entry
                navMeshMgr.get_collision_handler().sort_entries()
                return navMeshMgr.get_collision_handler().get_entry(0)
    return None

# toggle debug draw
def toggleDebugDraw():
    global toggleDebugFlag, navMesh
    if navMesh.toggle_debug_drawing(toggleDebugFlag) >= 0:
        toggleDebugFlag = not toggleDebugFlag

# toggle setup/cleanup
def toggleSetupCleanup():
    global navMesh, app, setupCleanupFlag, toggleDebugFlag, firstSetup
    if setupCleanupFlag:
        # true: setup
        navMesh.set_owner_node_path(sceneNP)
        navMesh.setup()
        navMesh.enable_debug_drawing(app.camera)
        #
        if firstSetup:
            # first set initial position and target
            NodePath.any_path(crowdAgent).set_pos(LPoint3f(0.0, 15.0, 10.0))
            # then attach the crowd agent to the nav mesh
            navMesh.add_crowd_agent(NodePath.any_path(crowdAgent))
            crowdAgent.set_move_target(LPoint3f(0.0, 20.0, 10.0))
            firstSetup = False

        # show debug draw
        navMesh.toggle_debug_drawing(True)
        toggleDebugFlag = False
        # show doors
        for ref in list(doorRefs):
            points = navMesh.get_convex_volume_by_ref(ref);
            if points.get_num_values() == 0:
                print("Door's invalid ref: " + str(ref) + " ...removing")
                doorRefs.remove(ref)
                continue
            centroid = LPoint3f(0,0,0)
            for p in points:
                centroid += p
            centroid /= points.get_num_values()
            settings = navMesh.get_convex_volume_settings(centroid)
 
            if not (settings == navMesh.get_convex_volume_settings(ref)):
                print("assertion failed: settings == navMesh.get_convex_volume_settings(ref)")
 
            print("Door n. " + str(doorRefs.index(ref)))
            print("\tref: " + str(settings.get_ref()) + " | "
                    "area: " + str(settings.get_area()) + " | "
                    "flags: " + str(settings.get_flags()))
    else:
        # false: cleanup
        navMesh.cleanup()
        pointList.clear()
        # now crowd agents and obstacles are detached:
        # prevent to make them disappear from the scene
        for agent in navMesh:
            NodePath.any_path(agent).reparent_to(app.render)
        for i in range(navMesh.get_num_obstacles()):
            ref = navMesh.get_obstacle(i)
            navMesh.get_obstacle_by_ref(ref).reparent_to(app.render)
        
    setupCleanupFlag = not setupCleanupFlag

# place crowd agent
def placeCrowdAgent():
    global navMesh, crowdAgent
    if navMesh == None or crowdAgent == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        # remove agent from nav mesh
        navMesh.remove_crowd_agent(NodePath.any_path(crowdAgent))
        point = entry0.get_surface_point(NodePath())
        NodePath.any_path(crowdAgent).set_pos(point)
        # re-add agent to nav mesh
        navMesh.add_crowd_agent(NodePath.any_path(crowdAgent))
        # just for debug draw the agent's found path
        navMesh.path_find_follow(point, crowdAgent.get_move_target())

# handle set move target
def setMoveTarget():
    global navMesh, crowdAgent
    if navMesh == None or crowdAgent == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        target = entry0.get_surface_point(NodePath())
        crowdAgent.set_move_target(target)
        # just for debug draw the agent's found path
        navMesh.path_find_follow(
                NodePath.any_path(crowdAgent).get_pos(), target)

if __name__ == '__main__':
    # Load your application's configuration
    load_prc_file_data("", "model-path " + dataDir)
    load_prc_file_data("", "win-size 1024 768")
    load_prc_file_data("", "show-frame-rate-meter #t")
    load_prc_file_data("", "sync-video #t")
        
    # Setup your application
    app = ShowBase()
       
    # # here is room for your own code
    # create a nav mesh manager
    navMesMgr = RNNavMeshManager(app.render, mask)

    # get a sceneNP as owner model
    sceneNP = app.loader.load_model("dungeon.egg")
    sceneNP.set_collide_mask(mask)
    sceneNP.reparent_to(app.render)
    
    # create a nav mesh and attach it to render
    navMeshNP = navMesMgr.create_nav_mesh()
    navMesh = navMeshNP.node()
    
    # mandatory: set sceneNP as owner of navMesh
    navMesh.set_owner_node_path(sceneNP)
    
    # reparent navMeshNP to a reference NodePath
    navMeshNP.reparent_to(app.render)
    
    # set nav mesh type
    navMesh.set_nav_mesh_type_enum(RNNavMesh.SOLO)
#     navMesh.set_nav_mesh_type_enum(RNNavMesh.TILE);
#     navMesh.set_nav_mesh_type_enum(RNNavMesh.OBSTACLE)
    
    # get the agent model
    agentNP = app.loader.load_model("eve.egg")
    agentNP.set_scale(0.40)

    # create the crowd agent and set the position
    crowdAgentNP = navMesMgr.create_crowd_agent("crowdAgent")
    crowdAgent = crowdAgentNP.node()
    
    # attach the agent model to crowdAgent
    agentNP.reparent_to(crowdAgentNP)

    # start the path finding default update task
    navMesMgr.start_default_update()

    # toggle setup (true) and cleanup (false)
    app.accept("s", toggleSetupCleanup)

    # toggle debug draw
    app.accept("d", toggleDebugDraw)

    # place crowd agent
    app.accept("p", placeCrowdAgent)

    # handle move target on scene surface
    app.accept("t", setMoveTarget)

    # add doors
    app.accept("a", addDoor, [True])
    app.accept("shift-a", addDoor, [False])

    # remove doors
    app.accept("r", removeDoor)

    # open/close door
    app.accept("o", openCloseDoor)
    
    # handle change speed
    app.accept("v", changeSpeed)

    # handle cycle queries
    app.accept("q", cycleQueries)

    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-10.0, 90.0, -12.0);
    trackball.set_hpr(0.0, 35.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

