'''
Created on Mar 24, 2016

@author: consultit
'''

import panda3d.core
from p3recastnavigation import RNNavMeshManager, RNNavMesh, ValueList_LPoint3f
from panda3d.core import load_prc_file_data, LPoint3f, BitMask32, NodePath, \
                LVecBase4f, TextNode
from direct.showbase.ShowBase import ShowBase

dataDir = "../data"
# global data
app = None
mask = BitMask32(0x10)
navMesh = None
crowdAgent = None
# models and animations
sceneNP = None
setupCleanupFlag = True
toggleDebugFlag = False
maxVel = 3.5
resetVel = True
query = 0
areaPointList = ValueList_LPoint3f()
areaRefs = []
linkPointPair = ValueList_LPoint3f()
linkRefs = []
firstSetup = True

# # functions' declarations and definitions

def changeSpeed():
    """ handle change speed"""
    
    global crowdAgent, resetVel, maxVel
    if crowdAgent == None:
        return
    
    ap = crowdAgent.get_params()
    if resetVel:
        ap.set_maxSpeed(0.0)
    else:
        ap.set_maxSpeed(maxVel)
    crowdAgent.set_params(ap)
    resetVel = not resetVel

def cycleQueries():
    """cycle over queries"""
    
    global query, navMesh, crowdAgent
    if (crowdAgent == None) or (navMesh == None):
        return
    
    crowdAgentNP = NodePath.any_path(crowdAgent)
    startPos, endPos = (crowdAgentNP.get_pos(), crowdAgent.get_move_target())
    if query == 0:
        print("get path find to follow and its cost:")
        print("\tfrom " + str(startPos) + " to " + str(endPos))
        areaPointList = navMesh.path_find_follow(startPos, endPos);
        for p in areaPointList:
            print("\t" + str(p))
        print("\tcost: " + str(navMesh.path_find_follow_cost(startPos, endPos)))
    elif query == 1: 
        print("get path find to follow straight:")
        print("\tfrom " + str(startPos) + " to " + str(endPos))
        pointFlagList = navMesh.path_find_straight(startPos,
                        endPos, RNNavMesh.NONE_CROSSINGS);
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
        print("check visibility:")
        print("\tfrom " + str(startPos) + " to " + str(endPos))
        hitPoint = navMesh.ray_cast(startPos, endPos)
        RES = ""
        if hitPoint != endPos:
            RES = "not "
        print("\thit " + str(hitPoint) + " : " + RES + "visible!")
    elif query == 3:
        print("get distance to wall:")
        print("\tfrom " + str(startPos))
        distance = navMesh.distance_to_wall(startPos)
        print("\t" + str(distance))    
    else:
        pass
    query += 1
    query = query % 4

def addArea(data):
    """add area's (convex volume) points"""
    
    global areaPointList, areaRefs, navMesh
    if navMesh == None:
        return
    
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        addPoint = data
        point = entry0.get_surface_point(NodePath())
        if addPoint:
            RNNavMeshManager.get_global_ptr().debug_draw_reset()
            # add to list
            areaPointList.add_value(point)
            RNNavMeshManager.get_global_ptr().debug_draw_primitive(
                    RNNavMeshManager.POINTS, areaPointList,
                    LVecBase4f(1.0, 0.0, 0.0, 1.0), 4.0)
            print(point)
        else:
            RNNavMeshManager.get_global_ptr().debug_draw_reset()
            # add last point to list
            areaPointList.add_value(point)
            print(point)
            # add convex volume (area)
            ref = navMesh.add_convex_volume(areaPointList, RNNavMesh.POLYAREA_DOOR)
            print("Added (temporary) area with ref: " + str(ref))
            areaRefs.append(ref)
            # reset list
            areaPointList.clear()

def removeArea():
    """remove an area (convex volume)"""
    
    global navMesh
    if navMesh == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to remove area
        ref = navMesh.remove_convex_volume(point)
        if ref >= 0:
            print("Removed area with ref: " + str(ref))

def addLink():
    """add a link's (off mesh connection) point pair"""
    
    global linkPointPair, linkRefs, navMesh
    if navMesh == None:
        return
    
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        if linkPointPair.get_num_values() == 0:
            RNNavMeshManager.get_global_ptr().debug_draw_reset()
            # add start point to list
            linkPointPair.add_value(point)
            RNNavMeshManager.get_global_ptr().debug_draw_primitive(
                    RNNavMeshManager.POINTS, linkPointPair,
                    LVecBase4f(0.0, 0.0, 1.0, 1.0), 4.0)
            print(point)
        else:
            RNNavMeshManager.get_global_ptr().debug_draw_reset()
            # add end point to list
            linkPointPair.add_value(point)
            print(point)
            # add off mesh connection (link)
            ref = navMesh.add_off_mesh_connection(linkPointPair, True)
            print("Added (temporary) bidirectional link with ref: " + str(ref))
            linkRefs.append(ref)
            # reset list
            linkPointPair.clear()

def removeLink():
    """remove a link (off mesh connection)"""
    
    global navMesh
    if navMesh == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to remove link
        ref = navMesh.remove_off_mesh_connection(point)
        if ref >= 0:
            print("Removed link with ref: " + str(ref))

def enableDisableArea():
    """enable disable area (convex volume)"""
    
    global navMesh
    if navMesh == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to get area'settings by inside point
        settings = navMesh.get_convex_volume_settings(point)
        if settings.get_ref() >= 0:
            if not (navMesh.get_convex_volume_settings(settings.get_ref()) == settings):
                return
            
            # found a area: check if enabled or disabled
            if settings.get_flags() & RNNavMesh.POLYFLAGS_DISABLED:
                # area is disabled (convex volume disabled): enable
                print("Open the area: ")
            else:
                # area is enabled (convex volume disabled): disable
                print("Close the area: ")
            # switch area enable/disable
            settings.set_flags(settings.get_flags() ^ RNNavMesh.POLYFLAGS_DISABLED)
            # update settings
            navMesh.set_convex_volume_settings(settings.get_ref(), settings)
            print("\tref: " + str(settings.get_ref()) + " | "
                    "area: " + str(settings.get_area()) + " | "
                    "flags: " + str(settings.get_flags()))   

def enableDisableLink():
    """enable disable link (off mesh connection)"""

    global navMesh
    if navMesh == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        point = entry0.get_surface_point(NodePath())
        # try to get link'settings by start/end point
        settings = navMesh.get_off_mesh_connection_settings(point)
        if settings.get_ref() >= 0:
            if not (navMesh.get_off_mesh_connection_settings(settings.get_ref()) == settings):
                return

            # found a link: check if enabled or disabled
            if settings.get_flags() & RNNavMesh.POLYFLAGS_DISABLED:
                # link is disabled (convex volume disabled): enable
                print("Enable the link: ")
            else:
                # link is enabled (convex volume disabled): disable
                print("Disable the link: ")
            # switch link enable/disable
            settings.set_flags(settings.get_flags() ^ RNNavMesh.POLYFLAGS_DISABLED)
            # update settings
            navMesh.set_off_mesh_connection_settings(settings.get_ref(), settings);
            print("\tref: " + str(settings.get_ref()) + " | "
                    "area: " + str(settings.get_area()) + " | "
                    "flags: " + str(settings.get_flags()))   

def getCollisionEntryFromCamera():
    """throws a ray and returns the first collision entry or nullptr"""
    
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

def toggleDebugDraw():
    """toggle debug draw"""
    
    global toggleDebugFlag, navMesh
    if navMesh.toggle_debug_drawing(toggleDebugFlag) >= 0:
        toggleDebugFlag = not toggleDebugFlag

def toggleSetupCleanup():
    """toggle setup/cleanup"""
    
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
            NodePath.any_path(crowdAgent).show()
            firstSetup = False

        # show debug draw
        navMesh.toggle_debug_drawing(True)
        toggleDebugFlag = False
        # show areas
        for ref in list(areaRefs):
            points = navMesh.get_convex_volume_by_ref(ref);
            if points.get_num_values() == 0:
                print("Area's invalid ref: " + str(ref) + " ...removing")
                areaRefs.remove(ref)
                continue
            centroid = LPoint3f(0, 0, 0)
            for p in points:
                centroid += p
            centroid /= points.get_num_values()
            settings = navMesh.get_convex_volume_settings(centroid)
 
            if not (settings == navMesh.get_convex_volume_settings(ref)):
                print("assertion failed: settings == navMesh.get_convex_volume_settings(ref)")
 
            print("Area n. " + str(areaRefs.index(ref)))
            print("\tref: " + str(settings.get_ref()) + " | "
                    "area: " + str(settings.get_area()) + " | "
                    "flags: " + str(settings.get_flags()))
        # show links
        for ref in list(linkRefs):
            points = navMesh.get_off_mesh_connection_by_ref(ref)
            if points.get_num_values() == 0:
                print("Link's invalid ref: " + str(ref) + " ...removing")
                linkRefs.remove(ref)
                continue
            settings = navMesh.get_off_mesh_connection_settings(points[0])

            if not (settings == navMesh.get_off_mesh_connection_settings(ref)):
                print("assertion failed: settings == navMesh.get_off_mesh_connection_settings(ref)")

            print("Link n. " + str(linkRefs.index(ref)))
            print("\tref: " + str(settings.get_ref()) + " | "
                    "rad: " + str(settings.get_rad()) + " | "
                    "bidir: " + str(settings.get_bidir()) + " | "
                    "area: " + str(settings.get_area()) + " | "
                    "flags: " + str(settings.get_flags()))
    else:
        # false: cleanup
        navMesh.cleanup()
        areaPointList.clear()
        linkPointPair.clear()
        # now crowd agents and obstacles are detached
        # from navMesh's NodePath, so we need to
        # prevent them to disappear from the scene:
        # reparent to navMeshNP's parent (i.e. commonNP)
        for agent in navMesh:
            NodePath.any_path(agent).reparent_to(NodePath.any_path(navMesh).get_parent())
        
    setupCleanupFlag = not setupCleanupFlag

def placeCrowdAgent():
    """place crowd agent"""
    
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

def setMoveTarget():
    """handle set move target"""
    
    global navMesh, crowdAgent
    if navMesh == None or crowdAgent == None:
        return

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        target = entry0.get_surface_point(NodePath())
        crowdAgent.set_move_target(target)

if __name__ == '__main__':
    # Load your application's configuration
    load_prc_file_data("", "model-path " + dataDir)
    load_prc_file_data("", "win-size 1024 768")
    load_prc_file_data("", "show-frame-rate-meter #t")
    load_prc_file_data("", "sync-video #t")
        
    # Setup your application
    app = ShowBase()
       
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            "Press \"s\" to switch setup/cleanup\n\n"
            "When nav mesh is not set up:\n"
            "\t- press \"a\" to add area (convex volume) points under mouse cursor\n"
            " \t(\"shift-a\" for last point)\n"
            "\t- press \"r\" to remove area under mouse cursor\n"
            "\t- press \"o\" to enable/disable area under mouse cursor\n"
            "\t- press \"l\" to add link (off mesh connection) points under mouse cursor)\n"
            "\t- press \"k\" to remove link with one of its points under mouse cursor\n"
            "\t- press \"i\" to enable/disable link with one of its points under mouse cursor\n\n"
            "When nav mesh is set up:\n"
            "\t- press \"d\" to switch debug drawing\n"
            "\t- press \"p\" to place agent under mouse cursor\n"
            "\t- press \"t\" to set agent's target under mouse cursor\n"
            "\t- press \"v\" to start/stop the agent\n"
            "\t- press \"q\" to cycle queries\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-0.1, 0.0, -0.42)
    textNodePath.set_scale(0.035)

    # create a nav mesh manager
    navMesMgr = RNNavMeshManager(app.render, mask)

    # reparent the reference node to render
    navMesMgr.get_reference_node_path().reparent_to(app.render)

    # get a sceneNP as owner model and reparent to the reference node
    sceneNP = app.loader.load_model("dungeon.egg")
    sceneNP.set_collide_mask(mask)
    sceneNP.reparent_to(navMesMgr.get_reference_node_path())
    
    # create a nav mesh (it is attached to the reference node)
    navMeshNP = navMesMgr.create_nav_mesh()
    navMesh = navMeshNP.node()
    
    # mandatory: set sceneNP as owner of navMesh
    navMesh.set_owner_node_path(sceneNP)
    
    # set nav mesh type
    navMesh.set_nav_mesh_type_enum(RNNavMesh.SOLO)
#     navMesh.set_nav_mesh_type_enum(RNNavMesh.TILE);
#     navMesh.set_nav_mesh_type_enum(RNNavMesh.OBSTACLE)

    # DEBUG DRAWING: make the debug reference node path sibling of the reference node
    navMesMgr.get_reference_node_path_debug().reparent_to(app.render)
    
    # get the agent model
    agentNP = app.loader.load_model("eve.egg")
    agentNP.set_scale(0.40)

    # create the crowd agent and set the position
    crowdAgentNP = navMesMgr.create_crowd_agent("crowdAgent")
    crowdAgentNP.hide()
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

    # add areas
    app.accept("a", addArea, [True])
    app.accept("shift-a", addArea, [False])
    # remove areas
    app.accept("r", removeArea)

    # add links
    app.accept("l", addLink)
    # remove links
    app.accept("k", removeLink)

    # enable/disable area
    app.accept("o", enableDisableArea)
    # enable/disable link
    app.accept("i", enableDisableLink)

    # open/close area
    app.accept("o", enableDisableArea)
    
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

