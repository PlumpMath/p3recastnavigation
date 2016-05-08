/**
 * \file rnCrowdAgent.I
 *
 * \date 2016-03-19
 * \author consultit
 */

#ifndef RNCROWDAGENT_I_
#define RNCROWDAGENT_I_

///RNCrowdAgent inline definitions

/**
 * Gets RNCrowdAgent parameters.
 */
INLINE RNCrowdAgentParams RNCrowdAgent::get_params()
{
	return mAgentParams;
}

/**
 * Gets RNCrowdAgent move target.
 */
INLINE LPoint3f RNCrowdAgent::get_move_target()
{
	return mMoveTarget;
}

/**
 * Gets RNCrowdAgent move velocity.
 */
INLINE LVector3f RNCrowdAgent::get_move_velocity()
{
	return mMoveVelocity;
}

/**
 * Gets RNCrowdAgent movement type (recast native or kinematic).
 */
INLINE RNCrowdAgent::RNCrowdAgentMovType RNCrowdAgent::get_mov_type() const
{
	return mMovType;
}

/**
 * Gets the RNNavMesh this RNCrowdAgent is added to.
 */
INLINE PT(RNNavMesh) RNCrowdAgent::get_nav_mesh() const
{
	return mNavMesh;
}

/**
 * Enables/disables the RNCrowdAgent event to be thrown.
 */
INLINE void RNCrowdAgent::enable_throw_event(RNEventThrown event,
		ThrowEventData eventData)
{
	do_enable_crowd_agent_event(event, eventData);
}

/**
 * Resets RNCrowdAgent.
 */
inline void RNCrowdAgent::do_reset()
{
	//
	mMovType = RECAST;
	mAgentIdx = -1;
	mAgentParams = RNCrowdAgentParams();
	mMoveTarget = LPoint3f::zero();
	mMoveVelocity = LVector3f::zero();
	mHeigthCorrection = LVector3f::zero();
	mMove = mSteady = ThrowEventData();
	mReferenceNP.clear();
}

INLINE ostream &operator <<(ostream &out, const RNCrowdAgent& crowdAgent)
{
	crowdAgent.output(out);
	return out;
}

#endif /* RNCROWDAGENT_I_ */
