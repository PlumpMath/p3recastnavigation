/**
 * \file rnCrowdAgent.h
 *
 * \date 2016-03-16
 * \author consultit
 */

#ifndef RNCROWDAGENT_H_
#define RNCROWDAGENT_H_

#include "rnNavMesh.h"
#include "rnNavMeshManager.h"
#include "rnTools.h"
#include "recastnavigation_includes.h"
#include "nodePath.h"

/**
 * Class implementing dtCrowdAgent from RecastNavigation nav mesh and path finding library.
 *
 * \see
 * 		- https://github.com/recastnavigation/recastnavigation.git
 * 		- http://digestingduck.blogspot.it
 * 		- https://groups.google.com/forum/?fromgroups#!forum/recastnavigation
 *
 * This object must be added to a RNNavMesh to be driven to its own target.\n
 * A model should be associated to this object.\n
 * This object could be of type:
 * - "recast" (the default): its movement/orientation follows strictly the
 * path as updated by RecastNavigation library
 * - "kinematic": its movement/orientation is corrected to stand on floor.
 * If specified in "thrown_events", this object can throw events which
 * actually are sent:
 * - on moving (default event name: <OwnerObjectName>_CrowdAgent_Move)
 * - on being steady (default event name: <OwnerObjectName>_CrowdAgent_Steady)
 * Events are thrown continuously at a frequency which is the minimum between
 * the fps and the frequency specified (which defaults to 30 times per seconds).\n
 * The argument of each event is a reference to this component.\n
 *
 * \note the node path of this object will be reparented (if necessary)
 * when added to a RNNavMesh, to the same reference node (i.e. parent) of
 * the RNNavMesh object.
 *
 * > **Manager Creation Parameter(s)**:
 * param | type | default | note
 * ------|------|---------|-----
 * | *thrown_events*				|single| - | specified as "event1@[event_name1]@[frequency1][:...[:eventN@[event_nameN]@[frequencyN]]]" with eventX = move,steady
 * | *add_to_navmesh*				|single| - | -
 * | *mov_type*						|single| *recast* | values: recast,kinematic
 * | *move_target*					|single| 0.0,0.0,0.0 | -
 * | *move_velocity*				|single| 0.0,0.0,0.0 | -
 * | *max_acceleration*			    |single| 8.0 | -
 * | *max_speed*					|single| 3.5 | -
 * | *collision_query_range*		|single| 12.0 | * RNNavMesh::agent_radius
 * | *path_optimization_range*		|single| 30.0 | * RNNavMesh::agent_radius
 * | *separation_weight* 			|single| 2.0 | -
 * | *update_flags*					|single| *0x1b* | -
 * | *obstacle_avoidance_type*		|single| *3* | values: 0,1,2,3
 *
 * \note parts inside [] are optional.\n
 */
class EXPORT_CLASS RNCrowdAgent: public PandaNode
{
PUBLISHED:
	/**
	 * RNCrowdAgent movement type.
	 */
	enum RNCrowdAgentMovType
	{
		RECAST,
		RECAST_KINEMATIC,
		AgentMovType_NONE
	};

	/**
	 * RNCrowdAgent thrown events.
	 */
	enum RNEventThrown
	{
		MOVE_EVENT,
		STEADY_EVENT
	};

	/**
	 * Equivalent to Detour UpdateFlags enum.
	 */
	enum RNUpdateFlags
	{
#ifndef CPPPARSER
		ANTICIPATE_TURNS = DT_CROWD_ANTICIPATE_TURNS,
		OBSTACLE_AVOIDANCE = DT_CROWD_OBSTACLE_AVOIDANCE,
		SEPARATION = DT_CROWD_SEPARATION,
		OPTIMIZE_VIS = DT_CROWD_OPTIMIZE_VIS, ///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
		OPTIMIZE_TOPO = DT_CROWD_OPTIMIZE_TOPO, ///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
#else
		ANTICIPATE_TURNS,OBSTACLE_AVOIDANCE,SEPARATION,
		OPTIMIZE_VIS,OPTIMIZE_TOPO,
#endif //CPPPARSER
	};

	/**
	 * Equivalent to Detour CrowdAgentState enum.
	 */
	enum RNCrowdAgentState
	{
#ifndef CPPPARSER
		STATE_INVALID = DT_CROWDAGENT_STATE_INVALID,	///< The agent is not in a valid state.
		STATE_WALKING = DT_CROWDAGENT_STATE_WALKING,	///< The agent is traversing a normal navigation mesh polygon.
		STATE_OFFMESH = DT_CROWDAGENT_STATE_OFFMESH,	///< The agent is traversing an off-mesh connection.
#else
		STATE_INVALID,STATE_WALKING,STATE_OFFMESH,
#endif //CPPPARSER
	};

	virtual ~RNCrowdAgent();

	int set_params(const RNCrowdAgentParams& agentParams);
	INLINE RNCrowdAgentParams get_params();
	int set_move_target(const LPoint3f& pos);
	INLINE LPoint3f get_move_target();
	int set_move_velocity(const LVector3f& vel);
	INLINE LVector3f get_move_velocity();
	void set_mov_type(RNCrowdAgentMovType movType);
	INLINE RNCrowdAgentMovType get_mov_type() const;

	LVector3f get_actual_velocity();
	RNCrowdAgentState get_traversing_state();
	INLINE PT(RNNavMesh) get_nav_mesh() const;

	INLINE void enable_throw_event(RNEventThrown event, ThrowEventData eventData);

	void output(ostream &out) const;

protected:
	friend class RNNavMeshManager;
	friend class RNNavMesh;

	RNCrowdAgent(const string& name);

private:
	///The RNNavMesh this RNCrowdAgent is added to.
	PT(RNNavMesh) mNavMesh;
	///The reference node path.
	NodePath mReferenceNP;
	///The movement type.
	RNCrowdAgentMovType mMovType;
	///The RNCrowdAgent index.
	int mAgentIdx;
	///The associated dtCrowdAgent data.
	///@{
	RNCrowdAgentParams mAgentParams;
	LPoint3f mMoveTarget;
	LVector3f mMoveVelocity;
	///@}
	///Height correction for kinematic RNCrowdAgents.
	LVector3f mHeigthCorrection;

	void do_reset();
	void do_initialize();
	void do_finalize();

	void do_update_pos_dir(float dt, const LPoint3f& pos, const LVector3f& vel);

	/**
	 * Throwing RNCrowdAgent events.
	 */
	///@{
	ThrowEventData mMove, mSteady;
	///Helper.
	void do_enable_crowd_agent_event(RNEventThrown event, ThrowEventData eventData);
	void do_throw_event(ThrowEventData& eventData);
	///@}

	// Explicitly disabled copy constructor and copy assignment operator.
	RNCrowdAgent(const RNCrowdAgent&);
	RNCrowdAgent& operator=(const RNCrowdAgent&);

	///TypedWritable API
public:
	static void register_with_read_factory();
	virtual void write_datagram(BamWriter *manager, Datagram &dg) override;
	virtual int complete_pointers(TypedWritable **plist, BamReader *manager) override;

protected:
	static TypedWritable *make_from_bam(const FactoryParams &params);
	virtual void fillin(DatagramIterator &scan, BamReader *manager) override;

	///TypedObject semantics: hardcoded
public:
	static TypeHandle get_class_type()
	{
		return _type_handle;
	}
	static void init_type()
	{
		PandaNode::init_type();
		register_type(_type_handle, "RNCrowdAgent", PandaNode::get_class_type());
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

INLINE ostream &operator << (ostream &out, const RNCrowdAgent & crowdAgent);

///inline
#include "rnCrowdAgent.I"

#endif /* RNCROWDAGENT_H_ */
