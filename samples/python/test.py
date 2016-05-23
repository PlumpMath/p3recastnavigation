'''
Created on Mar 24, 2016

@author: consultit
'''

# from direct.actor.Actor import Actor
import panda3d.core
from p3recastnavigation import RNNavMeshManager, RNNavMesh, ValueList_LPoint3f
from panda3d.core import load_prc_file_data, LPoint3f
from direct.showbase.ShowBase import ShowBase

dataDir = "../data"

mask = BitMask32(0x10)
navMesh = None
crowdAgent = None
sceneNP = None
setupCleanupFlag = True
toggleDebugFlag = True
halfVel = True
query = 0
area = 0
pointList = ValueList_LPoint3f()
doorRefs = []

def changeSpeed():
    global crowdAgent, halfVel
    if not crowdAgent:
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
    if not (crowdAgent and navMesh):
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
    global pointList, doorRefs, navMesh, area
    if not navMesh:
        return
    
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        addPoint = data
        point = entry0.get_surface_point(NodePath())
        if not addPoint:
            # add to list
            pointList.add_value(point)
            print(point)
        else:
            # add last point to list
            pointList.add_value(point)
            print(point)
            # add convex volume (door)
            ref = navMesh.add_convex_volume(pointList, RNNavMesh.POLYAREA_DOOR)
            print("Added door with (temporary) ref: " + str(ref))
            doorRefs.append(ref)
            # reset list
            pointList[:] = []

def removeDoor():
    global pointList, doorRefs, navMesh, area
    if not navMesh:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to remove door
        ref = navMesh.remove_convex_volume(point)
        if ref >= 0:
            print("Removed door with ref: " + str(ref))

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
    global navMesh, app, setupCleanupFlag, toggleDebugFlag
    if setupCleanupFlag:
        # true: setup
        navMesh.set_owner_node_path(sceneNP)
        navMesh.setup()
        navMesh.enable_debug_drawing(app.camera)
    else:
        # false: cleanup
        navMesh.cleanup()
        pointList[:] = []
        # now crowd agents and obstacles are detached:
        # prevent to make them disappear from the scene
        for agent in navMesh:
            NodePath.any_path(agent).reparent_to(app.render)
        for i in range(navMesh.get_num_obstacles()):
            ref = navMesh.get_obstacle(i)
            navMesh.get_obstacle_by_ref(ref).reparent_to(app.render)
        # reset debug draw flag
        toggleDebugFlag = true
    setupCleanupFlag = not setupCleanupFlag

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
    
    # setup the nav mesh with scene as its owner object
#     navMesh.setup()
    
    # get the agent model
#     agentNP = app.loader.load_model("eve.egg")
#     agentNP.set_scale(0.40)

    # create the crowd agent and set the position
#     crowdAgentNP = navMesMgr.create_crowd_agent("crowdAgent")
#     crowdAgent = crowdAgentNP.node()
#     crowdAgentNP.set_pos(24.0, -20.4, -2.37)
    
    # attach the agent model to crowdAgent
#     agentNP.reparent_to(crowdAgentNP)
    
    # attach the crowd agent to the nav mesh
#     navMesh.add_crowd_agent(crowdAgentNP)

    # start the path finding default update task
    navMesMgr.start_default_update()

    # enable debug draw
#     navMesh.enable_debug_drawing(app.camera)

    # toggle debug draw
#     navMesh.toggle_debug_drawing(True)

    # toggle setup (true) and cleanup (false)
    app.accept("s", toggleSetupCleanup)

    # toggle debug draw
    app.accept("d", toggleDebugDraw)
    
    # set crowd agent move target on scene surface
#     crowdAgent.set_move_target(LPoint3f(-20.5, 5.2, -2.36))

    # add doors
    app.accept("a", addDoor, [True])
    app.accept("shift-a", addDoor, [False])
    
    # remove doors
    app.accept("r", removeDoor)
    
    # handle change speed
    app.accept("v", changeSpeed)

    # handle cycle queries
    app.accept("q", cycleQueries)

    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-10.0, 90.0, -2.0);
    trackball.set_hpr(0.0, 15.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

