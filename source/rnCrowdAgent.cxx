/**
 * \file rnCrowdAgent.cpp
 *
 * \date 2016-03-16
 * \author consultit
 */

#include "rnCrowdAgent.h"

#include "rnNavMesh.h"
#include "throw_event.h"

/**
 *
 */
RNCrowdAgent::RNCrowdAgent(const string& name):PandaNode(name)
{
	mNavMesh.clear();
	do_reset();
}

/**
 *
 */
RNCrowdAgent::~RNCrowdAgent()
{
}

/**
 * Sets RNCrowdAgent parameters.
 */
int RNCrowdAgent::set_params(const RNCrowdAgentParams& agentParams)
{
	//return if crowdAgent doesn't belong to any mesh
	nassertr_always(mNavMesh, RN_NAVMESH_NULL)

	//request RNNavMesh to update move target for this RNCrowdAgent
	return mNavMesh->do_set_crowd_agent_params(this, agentParams);
}

/**
 * Sets RNCrowdAgent move target.
 */
int RNCrowdAgent::set_move_target(const LPoint3f& pos)
{
	//return if crowdAgent doesn't belong to any mesh
	nassertr_always(mNavMesh, RN_NAVMESH_NULL)

	//request RNNavMesh to update move target for this RNCrowdAgent
	return mNavMesh->do_set_crowd_agent_target(this, pos);
}

/**
 * Sets RNCrowdAgent move velocity.
 */
int RNCrowdAgent::set_move_velocity(const LVector3f& vel)
{
	//return if crowdAgent doesn't belong to any mesh
	nassertr_always(mNavMesh, RN_NAVMESH_NULL)

	//request RNNavMesh to update move target for this RNCrowdAgent
	return mNavMesh->do_set_crowd_agent_velocity(this, vel);
}

/**
 * Sets RNCrowdAgent movement type (recast native or kinematic).
 */
void RNCrowdAgent::set_mov_type(RNCrowdAgentMovType movType)
{
	//if there isn't a traverser only RECAST is allowed
	RNNavMeshManager::GetSingletonPtr()->get_collision_traverser() ?
			mMovType = movType : mMovType = RECAST;
}

/**
 * Gets RNCrowdAgent actual velocity.
 */
LVector3f RNCrowdAgent::get_actual_velocity()
{
	//return vector 0 if crowd agent doesn't belong to any mesh
	nassertr_always(mNavMesh, LVector3f::zero())

	return rnsup::RecastToLVecBase3f(
			mNavMesh->get_recast_crowd()->getAgent(mAgentIdx)->vel);
}

/**
 * Gets RNCrowdAgent traversing state.
 */
RNCrowdAgent::RNCrowdAgentState RNCrowdAgent::get_traversing_state()
{
	//return if crowdAgent doesn't belong to any mesh
	nassertr_always(mNavMesh, STATE_INVALID)

	return static_cast<RNCrowdAgentState>(mNavMesh->get_recast_crowd()->getAgent(
			mAgentIdx)->state);
}

/**
 * Allows a RNCrowdAgent to be initialized.
 */
void RNCrowdAgent::do_initialize()
{
	WPT(RNNavMeshManager)mTmpl = RNNavMeshManager::get_global_ptr();
	//set RNCrowdAgent parameters
	//register to navmesh objectId
	string navMeshName = mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("add_to_navmesh"));
	//mov type
	string movType = mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("mov_type"));
	if (movType == string("kinematic"))
	{
		set_mov_type(RECAST_KINEMATIC);
	}
	else
	{
		set_mov_type(RECAST);
	}
	//
	string param;
	unsigned int idx, valueNum;
	pvector<string> paramValuesStr;
	//move target
	param = mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("move_target"));
	paramValuesStr = parseCompoundString(param, ',');
	valueNum = paramValuesStr.size();
	if (valueNum < 3)
	{
		paramValuesStr.resize(3, "0.0");
	}
	for (idx = 0; idx < 3; ++idx)
	{
		mMoveTarget[idx] = STRTOF(paramValuesStr[idx].c_str(), NULL);
	}
	//move velocity
	param = mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("move_velocity"));
	paramValuesStr = parseCompoundString(param, ',');
	valueNum = paramValuesStr.size();
	if (valueNum < 3)
	{
		paramValuesStr.resize(3, "0.0");
	}
	for (idx = 0; idx < 3; ++idx)
	{
		mMoveVelocity[idx] = STRTOF(paramValuesStr[idx].c_str(), NULL);
	}
	//
	float value;
	int valueInt;
	//max acceleration
	value = STRTOF(mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("max_acceleration")).c_str(),
			NULL);
	mAgentParams.set_maxAcceleration(value >= 0.0 ? value : -value);
	//max speed
	value = STRTOF(mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("max_speed")).c_str(), NULL);
	mAgentParams.set_maxSpeed(value >= 0.0 ? value : -value);
	//collision query range
	value = STRTOF(
			mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("collision_query_range")).c_str(),
			NULL);
	mAgentParams.set_collisionQueryRange(value >= 0.0 ? value : -value);
	//path optimization range
	value = STRTOF(
			mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("path_optimization_range")).c_str(),
			NULL);
	mAgentParams.set_pathOptimizationRange(value >= 0.0 ? value : -value);
	//separation weight
	value = STRTOF(mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("separation_weight")).c_str(),
			NULL);
	mAgentParams.set_separationWeight(value >= 0.0 ? value : -value);
	//update flags
	valueInt = strtol(mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("update_flags")).c_str(),
			NULL, 0);
	mAgentParams.set_updateFlags(valueInt >= 0.0 ? valueInt : -valueInt);
	//obstacle avoidance type
	valueInt = strtol(
			mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("obstacle_avoidance_type")).c_str(),
			NULL, 0);
	mAgentParams.set_obstacleAvoidanceType(valueInt >= 0.0 ? valueInt : -valueInt);
	//thrown events
	string thrownEventsParam = mTmpl->get_parameter_value(RNNavMeshManager::CROWDAGENT, string("thrown_events"));
	//
	//set thrown events if any
	unsigned int idx1, valueNum1;
	pvector<string> paramValuesStr1, paramValuesStr2;
	if (thrownEventsParam != string(""))
	{
		//events specified
		//event1@[event_name1]@[frequency1][:...[:eventN@[event_nameN]@[frequencyN]]]
		paramValuesStr1 = parseCompoundString(thrownEventsParam, ':');
		valueNum1 = paramValuesStr1.size();
		for (idx1 = 0; idx1 < valueNum1; ++idx1)
		{
			//eventX@[event_nameX]@[frequencyX]
			paramValuesStr2 = parseCompoundString(paramValuesStr1[idx1], '@');
			if (paramValuesStr2.size() >= 3)
			{
				RNEventThrown event;
				ThrowEventData eventData;
				//get default name prefix
				string objectType = get_name();
				//get name
				string name = paramValuesStr2[1];
				//get frequency
				float frequency = STRTOF(paramValuesStr2[2].c_str(), NULL);
				if (frequency <= 0.0)
				{
					frequency = 30.0;
				}
				//get event
				if (paramValuesStr2[0] == "move")
				{
					event = MOVE_EVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_CrowdAgent_Move";
					}
				}
				else if (paramValuesStr2[0] == "steady")
				{
					event = STEADY_EVENT;
					//check name
					if (name == "")
					{
						//set default name
						name = objectType + "_CrowdAgent_Steady";
					}
				}
				else
				{
					//paramValuesStr2[0] is not a suitable event:
					//continue with the next event
					continue;
				}
				//set event data
				eventData.mEnable = true;
				eventData.mEventName = name;
				eventData.mTimeElapsed = 0;
				eventData.mFrequency = frequency;
				//enable the event
				do_enable_crowd_agent_event(event, eventData);
			}
		}
	}
	//clear all no more needed "Param" variables
	thrownEventsParam.clear();
	//1: get the input from xml
	//2: add settings for RNCrowdAgent
	//set params: already done
	//set RNNavMesh object (if any)
	// set this NodePath
	NodePath thisNP = NodePath::any_path(this);
	// set the collide mask to avoid hit with the nav mesh manager ray
	thisNP.set_collide_mask(~mTmpl->get_collide_mask() &
			thisNP.get_collide_mask());
	//3: add to RNNavMesh update if requested
	PT(RNNavMesh) navMesh = NULL;
	for (int index = 0;
			index < RNNavMeshManager::get_global_ptr()->get_num_nav_meshes();
			++index)
	{
		navMesh = DCAST(RNNavMesh,
				RNNavMeshManager::get_global_ptr()->get_nav_mesh(index).node());
		if (navMesh->get_name() == navMeshName)
		{
			navMesh->add_crowd_agent(thisNP);
			break;
		}
	}
}

/**
 * On destruction cleanup.
 * Gives a RNCrowdAgent the ability to do any required cleanup just
 * when being destroyed.
 */
void RNCrowdAgent::do_finalize()
{
	NodePath thisNP = NodePath::any_path(this);
	//Remove from RNNavMesh update (if previously added)
	//mNavMesh will be cleared during removing, so
	//remove through a temporary pointer
	PT(RNNavMesh)navMesh = mNavMesh;
	if(navMesh)
	{
		navMesh->remove_crowd_agent(thisNP);
	}
	//
	mNavMesh.clear();
	//remove this NodePath
	thisNP.remove_node();
	//
	do_reset();
}

/**
 * Updates position/velocity/orientation of the controlled object.
 *
 * This method is called exclusively by the update() method of the
 * (friend) RNNavMesh object this RNCrowdAgent is added to.
 */
void RNCrowdAgent::do_update_pos_dir(float dt, const LPoint3f& pos, const LVector3f& vel)
{
	// get the squared velocity module
	float velSquared = vel.length_squared();
	NodePath thisNP = NodePath::any_path(this);

	//update node path position
	LPoint3f updatedPos = pos;
	if ((mMovType == RECAST_KINEMATIC) and (velSquared > 0.0))
	{
		// get nav mesh manager
		WPT(RNNavMeshManager) navMeshMgr = RNNavMeshManager::get_global_ptr();
		// correct panda's Z: set the collision ray origin wrt collision root
		LPoint3f pOrig = navMeshMgr->get_collision_root().get_relative_point(
				mReferenceNP, pos) + mHeigthCorrection;
		// get the collision height wrt the parent node path: the nav mesh owner
		Pair<bool,float> gotCollisionZ = navMeshMgr->get_collision_height(pOrig,
				mReferenceNP);
		if (gotCollisionZ.get_first())
		{
			//updatedPos.z needs correction
			updatedPos.set_z(gotCollisionZ.get_second());
		}
	}
	thisNP.set_pos(updatedPos);

	//update node path direction & throw events
	if (velSquared > 0.0)
	{
		//update node path direction
		thisNP.heads_up(updatedPos - vel);

		//throw Move event (if enabled)
		if (mMove.mEnable)
		{
			do_throw_event(mMove);
		}
		//reset Steady event (if enabled and if thrown)
		if (mSteady.mEnable and mSteady.mThrown)
		{
			mSteady.mThrown = false;
			mSteady.mTimeElapsed = 0.0;
		}
	}
	else //vel.length_squared() == 0.0
	{
		//reset Move event (if enabled and if thrown)
		if (mMove.mEnable and mMove.mThrown)
		{
			mMove.mThrown = false;
			mMove.mTimeElapsed = 0.0;
		}
		//throw Steady event (if enabled)
		if (mSteady.mEnable)
		{
			do_throw_event(mSteady);
		}
	}
}

/**
 * Enables/disables event throwing.
 */
void RNCrowdAgent::do_enable_crowd_agent_event(RNEventThrown event,
		ThrowEventData eventData)
{
	//some checks
	nassertv_always(not eventData.mEventName.empty())

	if (eventData.mFrequency <= 0.0)
	{
		eventData.mFrequency = 30.0;
	}

	switch (event)
	{
	case MOVE_EVENT:
		if (mMove.mEnable != eventData.mEnable)
		{
			mMove = eventData;
			mMove.mTimeElapsed = 0;
		}
		break;
	case STEADY_EVENT:
		if (mSteady.mEnable != eventData.mEnable)
		{
			mSteady = eventData;
			mSteady.mTimeElapsed = 0;
		}
		break;
	default:
		break;
	}
}

/**
 * Throws the event(s).
 */
void RNCrowdAgent::do_throw_event(ThrowEventData& eventData)
{
	if (eventData.mThrown)
	{
		eventData.mTimeElapsed += ClockObject::get_global_clock()->get_dt();
		if (eventData.mTimeElapsed >= eventData.mPeriod)
		{
			//enough time is passed: throw the event
			throw_event(eventData.mEventName, EventParameter(this));
			//update elapsed time
			eventData.mTimeElapsed -= eventData.mPeriod;
		}
	}
	else
	{
		//throw the event
		throw_event(eventData.mEventName, EventParameter(this));
		eventData.mThrown = true;
	}
}

/**
 * Writes a sensible description of the RNCrowdAgent to the indicated output
 * stream.
 */
void RNCrowdAgent::output(ostream &out) const
{
	out << get_type() << " " << get_name();
}

//TypedWritable API
/**
 * Tells the BamReader how to create objects of type RNCrowdAgent.
 */
void RNCrowdAgent::register_with_read_factory()
{
	BamReader::get_factory()->register_factory(get_class_type(), make_from_bam);
}

/**
 * Writes the contents of this object to the datagram for shipping out to a
 * Bam file.
 */
void RNCrowdAgent::write_datagram(BamWriter *manager, Datagram &dg)
{
	PandaNode::write_datagram(manager, dg);

	///Name of this RNCrowdAgent.
	dg.add_string(get_name());

	///The movement type.
	dg.add_uint8((uint8_t) mMovType);
	///The associated dtCrowdAgent data.
	///@{
	mAgentParams.write_datagram(dg);
	mMoveTarget.write_datagram(dg);
	mMoveVelocity.write_datagram(dg);
	///@}

	///Throwing RNCrowdAgent events.
	mMove.write_datagram(dg);
	mSteady.write_datagram(dg);

	///The RNNavMesh this RNCrowdAgent is added to.
	manager->write_pointer(dg, mNavMesh);
}

/**
 * Receives an array of pointers, one for each time manager->read_pointer()
 * was called in fillin(). Returns the number of pointers processed.
 */
int RNCrowdAgent::complete_pointers(TypedWritable **p_list, BamReader *manager)
{
	int pi = PandaNode::complete_pointers(p_list, manager);

	///The RNNavMesh this RNCrowdAgent is added to.
	mNavMesh = DCAST(RNNavMesh, p_list[pi++]);

	return pi;
}

/**
 * This function is called by the BamReader's factory when a new object of
 * type RNCrowdAgent is encountered in the Bam file.  It should create the
 * RNCrowdAgent and extract its information from the file.
 */
TypedWritable *RNCrowdAgent::make_from_bam(const FactoryParams &params)
{
	// return NULL if RNNavMeshManager if doesn't exist
	nassertr_always(RNNavMeshManager::get_global_ptr(), NULL)

	// create a RNCrowdAgent with default parameters' values: they'll be restored later
	RNNavMeshManager::get_global_ptr()->set_parameters_defaults(
			RNNavMeshManager::CROWDAGENT);
	RNCrowdAgent *node =
			DCAST(RNCrowdAgent,
					RNNavMeshManager::get_global_ptr()->create_crowd_agent(
							"CrowdAgent").node());

	DatagramIterator scan;
	BamReader *manager;

	parse_params(params, scan, manager);
	node->fillin(scan, manager);

	return node;
}

/**
 * This internal function is called by make_from_bam to read in all of the
 * relevant data from the BamFile for the new RNCrowdAgent.
 */
void RNCrowdAgent::fillin(DatagramIterator &scan, BamReader *manager)
{
	PandaNode::fillin(scan, manager);

	///Name of this RNCrowdAgent. string mName;
	set_name(scan.get_string());

	///The movement type. Use setter because of additional check.
	set_mov_type((RNCrowdAgentMovType)scan.get_uint8());
	///The associated dtCrowdAgent data. No setters.
	///@{
	mAgentParams.read_datagram(scan);
	mMoveTarget.read_datagram(scan);
	mMoveVelocity.read_datagram(scan);
	///@}

	///Throwing RNCrowdAgent events.
	mMove.read_datagram(scan);
	mSteady.read_datagram(scan);

	///The RNNavMesh this RNCrowdAgent is added to.
	manager->read_pointer(scan);
}

//TypedObject semantics: hardcoded
TypeHandle RNCrowdAgent::_type_handle;
