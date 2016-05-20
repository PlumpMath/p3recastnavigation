'''
Created on Mar 24, 2016

@author: consultit
'''

# from direct.actor.Actor import Actor
import panda3d.core
from p3recastnavigation import RNNavMeshManager, RNNavMesh
from panda3d.core import load_prc_file_data, LPoint3f
from direct.showbase.ShowBase import ShowBase

dataDir = "../data"

crowdAgent = None
halfVel = True

def changeSpeed():
    global crowdAgent, halfVel
    ap = crowdAgent.get_params()
    vel = ap.get_maxSpeed()
    if halfVel:
        ap.set_maxSpeed(vel / 2.0)
    else:
        ap.set_maxSpeed(vel * 2.0)
    crowdAgent.set_params(ap)
    halfVel = not halfVel

if __name__ == '__main__':
    # Load your application's configuration
    load_prc_file_data("", "model-path " + dataDir)
    load_prc_file_data("", "win-size 1024 768")
    load_prc_file_data("", "show-frame-rate-meter #t")
    load_prc_file_data("", "sync-video #t")
        
    # Setup your application
    app = ShowBase()
       
    # # here is room for your own code
    print("create a nav mesh manager")
    navMesMgr = RNNavMeshManager()

    print("get a sceneNP as owner model")
    sceneNP = app.loader.load_model("nav_test.egg")
    
    print("create a nav mesh and attach it to render")
    navMeshNP = navMesMgr.create_nav_mesh()
    navMesh = navMeshNP.node()
    
    print("mandatory: set sceneNP as owner of navMesh")
    navMesh.set_owner_node_path(sceneNP)
    
    print("setup the nav mesh with scene as its owner object")
    navMesh.setup()
    
    print("reparent navMeshNP to a reference NodePath")
    navMeshNP.reparent_to(app.render)
    
    print("get the agent model")
    agentNP = app.loader.load_model("eve.egg")
    agentNP.set_scale(0.40)

    print("create the crowd agent and set the position")
    crowdAgentNP = navMesMgr.create_crowd_agent("crowdAgent")
    crowdAgent = crowdAgentNP.node()
    crowdAgentNP.set_pos(24.0, -20.4, -2.37)
    
    print("attach the agent model to crowdAgent")
    agentNP.reparent_to(crowdAgentNP)
    
    print("attach the crowd agent to the nav mesh")
    navMesh.add_crowd_agent(crowdAgentNP)

    print("start the path finding default update task")
    navMesMgr.start_default_update()

    print("enable debug draw")
    navMesh.enable_debug_drawing(app.camera)

    print("toggle debug draw")
    navMesh.toggle_debug_drawing(True)
    
    print("set crowd agent move target on scene surface")
    crowdAgent.set_move_target(LPoint3f(-20.5, 5.2, -2.36))
    
    # handle change speed
    app.accept("s", changeSpeed)

    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-10.0, 90.0, -2.0);
    trackball.set_hpr(0.0, 15.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

