/**
 * \file rnNavMeshManager.h
 *
 * \date 2016-03-29
 * \author consultit
 */

#ifndef RNNAVMESHMANAGER_H_
#define RNNAVMESHMANAGER_H_

#include "rnTools.h"
#include "recastnavigation_includes.h"
#include "collisionTraverser.h"
#include "collisionHandlerQueue.h"
#include "collisionRay.h"

class RNNavMesh;
class RNCrowdAgent;

/**
 * RNNavMeshManager Singleton class.
 *
 * Used for handling RNNavMeshes and RNCrowdAgents.
 */
class EXPORT_CLASS RNNavMeshManager: public TypedReferenceCount,
		public Singleton<RNNavMeshManager>
{
PUBLISHED:
	RNNavMeshManager(const NodePath& root = NodePath(),
			const CollideMask& mask = GeomNode::get_default_collide_mask());
	virtual ~RNNavMeshManager();

	// RNNavMeshes
	NodePath create_nav_mesh();
	bool destroy_nav_mesh(NodePath navMeshNP);
	NodePath get_nav_mesh(int index) const;
	INLINE int get_num_nav_meshes() const;
	MAKE_SEQ(get_nav_meshes, get_num_nav_meshes, get_nav_mesh);

	// RNCrowdAgents
	NodePath create_crowd_agent(const string& name);
	bool destroy_crowd_agent(NodePath crowdAgentNP);
	NodePath get_crowd_agent(int index) const;
	INLINE int get_num_crowd_agents() const;
	MAKE_SEQ(get_crowd_agents, get_num_crowd_agents, get_crowd_agent);

	/**
	 * The type of object for creation parameters.
	 */
	enum RNType
	{
		NAVMESH = 0,
		CROWDAGENT
	};

	ValueList<string> get_parameter_name_list(RNType type);
	void set_parameter_values(RNType type, const string& paramName, const ValueList<string>& paramValues);
	ValueList<string> get_parameter_values(RNType type, const string& paramName);
	void set_parameter_value(RNType type, const string& paramName, const string& value);
	string get_parameter_value(RNType type, const string& paramName);
	void set_parameters_defaults(RNType type);

	AsyncTask::DoneStatus update(GenericAsyncTask* task);
	void start_default_update();
	void stop_default_update();

	//Get singleton
	INLINE static RNNavMeshManager* get_global_ptr();

	//Utilities
	float get_bounding_dimensions(NodePath modelNP, LVecBase3f& modelDims,
			LVector3f& modelDeltaCenter);
	Pair<bool,float> get_collision_height(const LPoint3f& origin,
			const NodePath& space = NodePath());
	INLINE CollideMask get_collide_mask();
	INLINE NodePath get_collision_root();
	INLINE CollisionTraverser* get_collision_traverser();
	INLINE CollisionHandlerQueue* get_collision_handler();
	INLINE CollisionRay* get_collision_ray();

	//serialization
	bool write_to_bam_file(const string& fileName);
	bool read_from_bam_file(const string& fileName);

private:
	///List of RNNavMeshes handled by this template.
	typedef pvector<PT(RNNavMesh)> NavMeshList;
	NavMeshList mNavMeshes;
	///RNNavMeshes' parameter table.
	ParameterTable mNavMeshesParameterTable;

	///List of RNCrowdAgents handled by this template.
	typedef pvector<PT(RNCrowdAgent)> CrowdAgentList;
	CrowdAgentList mCrowdAgents;
	///RNCrowdAgents' parameter table.
	ParameterTable mCrowdAgentsParameterTable;

	///@{
	///A task data for step simulation update.
	PT(TaskInterface<RNNavMeshManager>::TaskData) mUpdateData;
	PT(AsyncTask) mUpdateTask;
	///@}

	///Utilities.
	NodePath mRoot;
	CollideMask mMask; //a.k.a. BitMask32
	CollisionTraverser* mCTrav;
	CollisionHandlerQueue* mCollisionHandler;
	CollisionRay* mPickerRay;

	///TypedObject semantics: hardcoded
public:
	static TypeHandle get_class_type()
	{
		return _type_handle;
	}
	static void init_type()
	{
		TypedReferenceCount::init_type();
		register_type(_type_handle, "RNNavMeshManager",
				TypedReferenceCount::get_class_type());
	}
	virtual TypeHandle get_type() const override
	{
		return get_class_type();
	}
	virtual TypeHandle force_init_type() override
	{
		init_type();
		return get_class_type();
	}

private:
	static TypeHandle _type_handle;

};

///inline
#include "rnNavMeshManager.I"

#endif /* RNNAVMESHMANAGER_H_ */
