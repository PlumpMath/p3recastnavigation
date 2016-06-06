'''
Created on Mar 24, 2016

@author: consultit
'''

from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor
from panda3d.core import load_prc_file_data, NodePath, ClockObject, \
                BitMask32, LVector3f, LVecBase3f, LPoint3f, \
                AnimControlCollection, auto_bind, TextNode, TransformState, \
                BamFile, Filename
from p3recastnavigation import RNNavMeshManager, RNCrowdAgent, \
                ValueList_string
from panda3d.bullet import *
import random, sys

dataDir = "../data"
# global data
mask = BitMask32(0x10);
navMesh = None
NUMAGENTS = 2
crowdAgent = [None, None]
# models and animations
sceneNP = None
agentNP = [None, None]
sceneFile = "nav_test.egg"
agentFile = ["eve.egg", "ralph.egg"]
agentAnimFiles = [["eve-walk.egg", "eve-offbalance.egg"],
                  ["ralph-walk.egg", "ralph-offbalance.egg"]]
rateFactor = 1.50;
agentAnimCtls = [[None, None], [None, None]]
# obstacle model
obstacleFile = "plants2.egg"
# bame file
bamFileName = "nav_mesh.boo"

boxProxyBulletObstacleNP = None
boxBulletNP = None
boxBulletObstacleAdded = False
def hitBulletOstacle():
    global boxBulletNP 
    ix = random.uniform(0.0, 1.2) - 0.6 
    iy = random.uniform(0.0, 1.2) - 0.6
    boxBulletNP.node().set_active(True)
    boxBulletNP.node().apply_central_impulse(LVector3f(ix, iy, 5.0))

# # functions' declarations and definitions
 
def loadAllScene():
    """load all scene stuff"""
    
    global app, navMesh, crowdAgent, sceneNP, agentNP, bulletWorld, \
            boxProxyBulletObstacleNP, boxBulletNP
    navMesMgr = RNNavMeshManager.get_global_ptr()
    
    # get a sceneNP as owner model
    sceneNP = getOwnerModel()
    # set name: to ease restoring from bam file
    sceneNP.set_name("Owner")
   
    # create a nav mesh; its parent is the reference node
    navMeshNP = navMesMgr.create_nav_mesh()
    navMesh = navMeshNP.node()
    
    # mandatory: set sceneNP as owner of navMesh
    navMesh.set_owner_node_path(sceneNP)
    
    # setup the nav mesh with scene as its owner object
    navMesh.setup()
    
    # BULLET
    # create trimesh shape and add to a rigid body
    geomNodes = sceneNP.find_all_matches("**/+GeomNode")
    triMesh = BulletTriangleMesh()
    for i in range(geomNodes.get_num_paths()):
        geomNode = geomNodes.get_path(i).node()
        ts = geomNode.get_transform().compose(
            TransformState.make_scale(sceneNP.get_net_transform().get_scale()))
        geoms = geomNode.get_geoms()
        for geom in geoms:
            triMesh.add_geom(geom, True, ts)
    shape = BulletTriangleMeshShape(triMesh, False)
    node = BulletRigidBodyNode('sceneNP')
    node.add_shape(shape)
    # reparent to reference node and attach to bullet bulletWorld
    sceneBulletNP = navMesMgr.get_reference_node_path().attach_new_node(node)
    bulletWorld.attach_rigid_body(node)
    # Box obstacle
    shape = BulletBoxShape(LVector3f(0.5, 0.5, 0.5))
    node = BulletRigidBodyNode('Box')
    node.set_mass(1.0)
    node.set_deactivation_enabled(True)
    node.add_shape(shape)
    # reparent to reference node and attach to bullet bulletWorld
    boxBulletNP = navMesMgr.get_reference_node_path().attach_new_node(node)
    boxBulletNP.set_pos(-13.8, -10.1, 5.0)
    bulletWorld.attach_rigid_body(node)
    # attach a model
    model = loader.load_model('models/box.egg')
    model.flatten_light()
    model.set_pos(-0.5, -0.5, -0.5)
    model.reparent_to(boxBulletNP)
    boxProxyBulletObstacleNP = NodePath("proxy obstacle")
    model.instance_to(boxProxyBulletObstacleNP)
    boxProxyBulletObstacleNP.reparent_to(navMeshNP)
    boxProxyBulletObstacleNP.hide()

    # bullet debug
#     debugNode = BulletDebugNode('Debug')
#     debugNode.show_wireframe(True)
#     debugNode.show_constraints(True)
#     debugNode.show_bounding_boxes(False)
#     debugNode.show_normals(False)
#     debugNP = navMesMgr.get_reference_node_path().attach_new_node(debugNode)
#     debugNP.show()
#     bulletWorld.set_debug_node(debugNP.node())

    # reparent sceneNP to the reference node
    sceneNP.reparent_to(navMesMgr.get_reference_node_path())

    # reparent the reference node to render
    navMesMgr.get_reference_node_path().reparent_to(app.render)
        
    # get agentNP[] (and agentAnimNP[]) as models for crowd agents
    getAgentModelAnims()
    
    # create crowd agents and attach agentNP[] (and agentAnimNP[]) as children
    for i in range(NUMAGENTS):
        # set parameter for crowd agent's type (RECAST or RECAST_KINEMATIC)
        if i % 2 == 0:
            agentType = "recast"
        else:
            agentType = "kinematic"
        navMesMgr.set_parameter_value(RNNavMeshManager.CROWDAGENT, "mov_type", agentType)
        # create the crowd agent
        crowdAgentNP = navMesMgr.create_crowd_agent("crowdAgent" + str(i))
        crowdAgent[i] = crowdAgentNP.node()
        # set the position randomly
        randPos = getRandomPos(sceneNP)
        crowdAgentNP.set_pos(randPos)
        # attach some geometry (a model) to crowdAgent
        agentNP[i].reparent_to(crowdAgentNP)
        # attach the crowd agent to the nav mesh 
        # (crowdAgentNP is automatically reparented to navMeshNP) 
        navMesh.add_crowd_agent(crowdAgentNP)
        
def restoreAllScene():
    """restore all scene stuff  when read from bam file"""
    
    global navMesh, crowdAgent, sceneNP, agentAnimCtls
    # restore nav mesh: through nav mesh manager
    navMeshNP = RNNavMeshManager.get_global_ptr().get_nav_mesh(0)
    navMesh = navMeshNP.node()
    # restore sceneNP: through panda3d
    sceneNP = RNNavMeshManager.get_global_ptr().get_reference_node_path().find("**/Owner")
    # reparent the reference node to render
    RNNavMeshManager.get_global_ptr().get_reference_node_path().reparent_to(app.render)
    
    # restore crowd agents
    for i in range(NUMAGENTS):
        # restore the crowd agent: through nav mesh manager
        crowdAgentNP = RNNavMeshManager.get_global_ptr().get_crowd_agent(i)
        crowdAgent[i] = crowdAgentNP.node()
        # restore animations
        tmpAnims = AnimControlCollection()
        auto_bind(crowdAgent[i], tmpAnims)
        for j in range(tmpAnims.get_num_anims()):
            agentAnimCtls[i][j] = tmpAnims.get_anim(j)

def getOwnerModel():
    """loads the owner model"""
    
    global app, sceneFile, mask
    # get a model to use as nav mesh' owner object
    modelNP = app.loader.load_model(sceneFile)
    modelNP.set_collide_mask(mask)
#     modelNP.set_pos(5.0, 20.0, 5.0)
#     modelNP.set_h(30.0)
#     modelNP.set_scale(2.0)
    return modelNP

def getAgentModelAnims():
    """load the agents' models and anims"""
    
    global app, agentNP, agentFile
    # get some models, with animations, to attach to crowd agents
    for i in range(NUMAGENTS):
        # get the model
        agentNP[i] = app.loader.load_model(agentFile[i])
        # set random scale (0.35 - 0.45)
        scale = 0.35 + 0.1 * random.uniform(0.0, 1.0)
        agentNP[i].set_scale(scale)
        # associate an anim with a given anim control
        tmpAnims = AnimControlCollection()
        agentAnimNP = [None, None];
        # first anim -> modelAnimCtls[i][0]
        agentAnimNP[0] = app.loader.load_model(agentAnimFiles[i][0])
        agentAnimNP[0].reparent_to(agentNP[i])
        auto_bind(agentNP[i].node(), tmpAnims)
        agentAnimCtls[i][0] = tmpAnims.get_anim(0)
        tmpAnims.clear_anims()
        agentAnimNP[0].detach_node()
        # second anim -> modelAnimCtls[i][1]
        agentAnimNP[1] = app.loader.load_model(agentAnimFiles[i][1])
        agentAnimNP[1].reparent_to(agentNP[i])
        auto_bind(agentNP[i].node(), tmpAnims)
        agentAnimCtls[i][1] = tmpAnims.get_anim(0)
        tmpAnims.clear_anims()
        agentAnimNP[1].detach_node()
        # reparent all node paths
        agentAnimNP[0].reparent_to(agentNP[i])
        agentAnimNP[1].reparent_to(agentNP[i])

def readFromBamFile(fileName):
    """read scene from a file"""
    
    return RNNavMeshManager.get_global_ptr().read_from_bam_file(fileName)

def writeToBamFileAndExit(fileName):
    """write scene to a file (and exit)"""
    
    RNNavMeshManager.get_global_ptr().write_to_bam_file(fileName)
    #
    sys.exit(0)

def printCreationParameters():
    """print creation parameters"""
    
    navMesMgr = RNNavMeshManager.get_global_ptr()
    #
    valueList = navMesMgr.get_parameter_name_list(RNNavMeshManager.NAVMESH)
    print("\n" + "RNNavMesh creation parameters:")
    for name in valueList:
        print ("\t" + name + " = " + 
               navMesMgr.get_parameter_value(RNNavMeshManager.NAVMESH, name))
    #
    valueList = navMesMgr.get_parameter_name_list(RNNavMeshManager.CROWDAGENT)
    print("\n" + "RNCrowdAgent creation parameters:")
    for name in valueList:
        print ("\t" + name + " = " + 
               navMesMgr.get_parameter_value(RNNavMeshManager.CROWDAGENT, name))

def setParametersBeforeCreation():
    """set parameters as strings before nav meshes/crowd agents creation"""
    
    navMesMgr = RNNavMeshManager.get_global_ptr()
    # tweak some nav mesh parameter
    navMesMgr.set_parameter_value(RNNavMeshManager.NAVMESH, "navmesh_type",
            "obstacle")
    navMesMgr.set_parameter_value(RNNavMeshManager.NAVMESH, "build_all_tiles",
            "true")
    navMesMgr.set_parameter_value(RNNavMeshManager.NAVMESH, "agent_max_climb",
            "2.5")
    navMesMgr.set_parameter_value(RNNavMeshManager.NAVMESH, "agent_radius",
            "1.0");
    # change an area flags cost (tricky because multi-valued)
    valueList = navMesMgr.get_parameter_values(RNNavMeshManager.NAVMESH,
            "area_flags_cost")
    valueList.remove_value("1@0x02@10.0")
    valueList.add_value("1@0x02@100.0")
    navMesMgr.set_parameter_values(RNNavMeshManager.NAVMESH, "area_flags_cost",
            valueList)

    valueList = ValueList_string()
    # set some off mesh connections: "area_type@flag1[:flag2...:flagN]@cost"
    valueList.add_value("31.6,24.5,-2.0:20.2,9.4,-2.4@true")
    valueList.add_value("21.1,-4.5,-2.4:32.3,-3.0,-1.5@true")
    valueList.add_value("1.2,-13.1,15.2:11.8,-18.3,10.0@true")
    navMesMgr.set_parameter_values(RNNavMeshManager.NAVMESH,
            "offmesh_connection", valueList)
    # set some convex volumes: "x1,y1,z1[:x2,y2,z2...:xN,yN,zN]@area_type"
    valueList.clear()
    valueList.add_value(
            "-15.2,-22.9,-2.4:-13.4,-22.6,-2.4:-13.1,-26.5,-2.4:-16.4,-26.4,-2.7@1")
    navMesMgr.set_parameter_values(RNNavMeshManager.NAVMESH, "convex_volume",
            valueList)

    # set crowd agent throwing events
    valueList.clear()
    valueList.add_value("move@move-event@0.5")
    navMesMgr.set_parameter_values(RNNavMeshManager.CROWDAGENT,
            "thrown_events", valueList)
    #
    printCreationParameters()

def toggleDebugDraw():
    """toggle debug draw"""
    
    global toggleDebugFlag, navMesh
    toggleDebugFlag = not toggleDebugFlag
    navMesh.toggle_debug_drawing(toggleDebugFlag)

def toggleSetupCleanup():
    """toggle setup/cleanup"""
    
    global navMesh, app, setupCleanupFlag
    if setupCleanupFlag:
        # true: setup
        navMesh.set_owner_node_path(sceneNP)
        navMesh.setup()
        navMesh.enable_debug_drawing(app.camera)
        #
        app.taskMgr.add(updateNavMesh, "updateNavMesh", extraArgs=[navMesh])
    else:
        app.taskMgr.remove("updateNavMesh")
        # false: cleanup
        navMesh.cleanup()
    setupCleanupFlag = not setupCleanupFlag

def handleCrowdAgentEvent(crowAgent):
    """handle crowd agent's events"""
    
    agent = NodePath.any_path(crowAgent)
    print ("move-event - " + agent.get_name() + " - " + str(agent.get_pos()))

def placeCrowdAgents():
    """place crowd agents randomly"""
    
    global navMesh, sceneNP, crowdAgent
    for i in range(NUMAGENTS):
        # remove agent from nav mesh
        navMesh.remove_crowd_agent(NodePath.any_path(crowdAgent[i]))
        # set its random position
        randPos = getRandomPos(sceneNP)
        NodePath.any_path(crowdAgent[i]).set_pos(randPos)
        # re-add agent to nav mesh
        navMesh.add_crowd_agent(NodePath.any_path(crowdAgent[i]))

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

def setMoveTarget(agent):
    """handle set move target"""
    
    global navMesh
    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        target = entry0.get_surface_point(NodePath())
        agent.set_move_target(target)

def handleObstacles(data):
    """handle add/remove obstacles"""
    
    global navMesh, app, mask
    addObstacle = data
    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        # get the hit object
        hitObject = entry0.get_into_node_path()
        # check if hitObject is the nav mesh owner object or
        # this last one is one of its anchestors
        if addObstacle and ((hitObject == navMesh.get_owner_node_path())
                        or (navMesh.get_owner_node_path().is_ancestor_of(
                                hitObject))):
            # the hit object is the scene
            # add an obstacle to the scene
            # get a model as obstacle
            obstacleNP = app.loader.load_model(obstacleFile)
            obstacleNP.set_collide_mask(mask)
            # set random scale (0.01 - 0.02)
            scale = 0.01 + 0.01 * random.uniform(0.0, 1.0)
            obstacleNP.set_scale(scale);
            # set obstacle position
            pos = entry0.get_surface_point(sceneNP)
            obstacleNP.set_pos(sceneNP, pos)
            # try to add to nav mesh
            if navMesh.add_obstacle(obstacleNP) < 0:
                # something went wrong remove from scene
                obstacleNP.remove_node()
        # check if hitObject is not the nav mesh owner object and
        # this last one is not one of its anchestors
        elif (not addObstacle) and ((hitObject != navMesh.get_owner_node_path())
                        and (not navMesh.get_owner_node_path().is_ancestor_of(
                                hitObject))):
            # cycle the obstacle list
            for index in range(navMesh.get_num_obstacles()):
                ref = navMesh.get_obstacle(index)
                obstacleNP = navMesh.get_obstacle_by_ref(ref)
                # check if the hitObject == obstacle or
                # obstacle is an ancestor of the hitObject
                if (hitObject == obstacleNP) or (obstacleNP.is_ancestor_of(hitObject)):
                    # try to remove from nav mesh
                    if navMesh.remove_obstacle(obstacleNP) >= 0:
                        # all ok remove from scene
                        obstacleNP.remove_node();
                        hitObject.remove_node();
                        break;

def updateNavMesh(navMesh, task):
    """custom path finding update task to correct panda's Z to stay on floor"""
    
    global crowdAgent, bulletWorld, boxProxyBulletObstacleNP, \
            boxBulletObstacleAdded, boxBulletNP
    # call update for navMesh
    dt = ClockObject.get_global_clock().get_dt()
    navMesh.update(dt)
    # handle crowd agents' animation
    for i in range(NUMAGENTS):
        # get current velocity size
        currentVelSize = crowdAgent[i].get_actual_velocity().length()
        if currentVelSize > 0.0:
            # walk
            agentAnimCtls[i][0].set_play_rate(currentVelSize / rateFactor)
            if not agentAnimCtls[i][0].is_playing():
                agentAnimCtls[i][0].loop(True)
        else:
            # check if crowd agent is on off mesh connection
            if crowdAgent[i].get_traversing_state() == RNCrowdAgent.STATE_OFFMESH:
                # off-balance
                if not agentAnimCtls[i][1].is_playing():
                    agentAnimCtls[i][1].loop(True)
            else:
                # stop any animation
                agentAnimCtls[i][0].stop()
                agentAnimCtls[i][1].stop()
    #
    bulletWorld.doPhysics(dt)
    if (not boxBulletObstacleAdded) and (not boxBulletNP.node().is_active()):
        pos = boxBulletNP.get_pos(NodePath.any_path(navMesh)) + LVecBase3f(0.0, 0.0, -0.5)
        boxProxyBulletObstacleNP.set_pos(pos)
        navMesh.add_obstacle(boxProxyBulletObstacleNP)
        boxBulletObstacleAdded = True
    elif boxBulletObstacleAdded and boxBulletNP.node().is_active():
        navMesh.remove_obstacle(boxProxyBulletObstacleNP)
        boxBulletObstacleAdded = False
    #
    return task.cont

def getRandomPos(modelNP):
    """return a random point on the facing upwards surface of the model"""
    
    # collisions are made wrt render
    navMeshMgr = RNNavMeshManager.get_global_ptr()
    # get the bounding box of scene
    modelDims, modelDeltaCenter = (LVecBase3f(), LVector3f())
    # modelRadius not used
    navMeshMgr.get_bounding_dimensions(modelNP, modelDims, modelDeltaCenter)
    # throw a ray downward from a point with z = double scene's height
    # and x,y randomly within the scene's (x,y) plane
    # set the ray origin at double of maximum height of the model
    zOrig = ((-modelDeltaCenter.get_z() + modelDims.get_z() / 2.0) + modelNP.get_z()) * 2.0
    while True:
        x = modelDims.get_x() * (random.uniform(0.0, 1.0) - 0.5) - modelDeltaCenter.get_x() + modelNP.get_x()
        y = modelDims.get_y() * (random.uniform(0.0, 1.0) - 0.5) - modelDeltaCenter.get_y() + modelNP.get_y()
        gotCollisionZ = navMeshMgr.get_collision_height(LPoint3f(x, y, zOrig))
        if gotCollisionZ.get_first():
            break
    return LPoint3f(x, y, gotCollisionZ.get_second())

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
            "- press \"d\" to toggle debug drawing\n"
            "- press \"s\" to toggle setup cleanup\n"
            "- press \"p\" to place agents randomly\n"
            "- press \"t\", \"y\" to set agents' targets under mouse cursor\n"
            "- press \"o\" to add obstacle under mouse cursor\n"
            "- press \"shift-o\" to remove obstacle under mouse cursor\n");
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, 0.9)
    textNodePath.set_scale(0.035)
    
    # create a nav mesh manager; set root and mask to manage 'kinematic' agents
    navMesMgr = RNNavMeshManager(app.render, mask)

    # print creation parameters: defult values
    print("\n" + "Default creation parameters:")
    printCreationParameters()

    # set creation parameters as strings before nav meshes/crowd agent creation
    print("\n" + "Current creation parameters:")
    setParametersBeforeCreation()

    # setup bullet bulletWorld
    bulletWorld = BulletWorld()
    bulletWorld.setGravity(LVector3f(0, 0, -9.81))

    # load or restore all scene stuff: if passed an argument
    # try to read it from bam file
    if (not len(sys.argv) > 1) or (not readFromBamFile(sys.argv[1])):
        # no argument or no valid bamFile
        loadAllScene()
    else:
        # valid bamFile
        restoreAllScene()

    # show the added agents
    print("Agents added to nav mesh:")
    for agent in navMesh:
        print("\t- " + str(agent))
   
    # # first option: start the path finding default update task
#     navMesMgr.start_default_update()

    # # second option: start a path finding custom update task
    app.taskMgr.add(updateNavMesh, "updateNavMesh", extraArgs=[navMesh], appendTask=True)

    # DEBUG DRAWING
    # make the debug reference node path sibling of the reference node
    navMesMgr.get_reference_node_path_debug().reparent_to(app.render)
    # enable debug drawing
    navMesh.enable_debug_drawing(app.camera)

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw)
    
    # toggle setup (true) and cleanup (false)
    setupCleanupFlag = False
    app.accept("s", toggleSetupCleanup)

    # handle CrowdAgents' events
    app.accept("move-event", handleCrowdAgentEvent)

    # place crowd agents randomly
    app.accept("p", placeCrowdAgents)

    # handle move targets on scene surface
    app.accept("t", setMoveTarget, [crowdAgent[0]])
    app.accept("y", setMoveTarget, [crowdAgent[1]])

    # handle obstacle addition
    app.accept("o", handleObstacles, [True])

    # handle obstacle removal
    app.accept("shift-o", handleObstacles, [False]);
    
    # handle hit bullet proxy obstacle
    app.accept("b", hitBulletOstacle)
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-10.0, 90.0, -2.0);
    trackball.set_hpr(0.0, 15.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

