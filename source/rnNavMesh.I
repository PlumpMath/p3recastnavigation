/**
 * \file rnNavMesh.I
 *
 * \date 2016-03-19
 * \author consultit
 */

#ifndef RNNAVMESH_I_
#define RNNAVMESH_I_


///RNNavMesh inline definitions

/**
 * Sets the owner object's NodePath of this RNNavMesh.
 * Should be called before RNNavMesh setup.
 */
INLINE void RNNavMesh::set_owner_node_path(const NodePath& ownerObject)
{
	CONTINUE_IF_ELSE_V(! mNavMeshType)

	mOwnerObject = ownerObject;
}

/**
 * Returns the owner object's NodePath of this RNNavMesh.
 */
INLINE NodePath RNNavMesh::get_owner_node_path()const
{
	return mOwnerObject;
}

/**
 * Sets the reference NodePath of this RNNavMesh.
 */
INLINE void RNNavMesh::set_reference_node_path(const NodePath& reference)
{
	mReferenceNP = reference;
}

/**
 * Returns the owner object's input geometry min bounds.
 * or LVecBase3f::zero() before nav mesh setup.
 */
INLINE LVecBase3f RNNavMesh::get_recast_bounds_min() const
{
	return (mGeom ?
			rnsup::RecastToLVecBase3f(mGeom->getMeshBoundsMin()) :
			LVecBase3f::zero());
}

/**
 * Returns the owner object's input geometry max bounds.
 * or LVecBase3f::zero() before nav mesh setup.
 */
INLINE LVecBase3f RNNavMesh::get_recast_bounds_max() const
{
	return (mGeom ?
			rnsup::RecastToLVecBase3f(mGeom->getMeshBoundsMax()) :
			LVecBase3f::zero());
}

/**
 * Returns the underlying NavMeshType type: SOLO, TILE, OBSTACLES.
 */
INLINE RNNavMesh::RNNavMeshTypeEnum RNNavMesh::get_nav_mesh_type_enum() const
{
	return mNavMeshTypeEnum;
}

/**
 * Returns the underlying NavMeshType settings.
 */
INLINE RNNavMeshSettings RNNavMesh::get_nav_mesh_settings() const
{
	return mNavMeshSettings;
}

/**
 * Sets the area's 'ored' flags.
 * Should be called before RNNavMesh setup.
 * \note oredFlags: by default flags as specified in RNNavMeshPolyFlagsEnum.
 */
INLINE void RNNavMesh::set_area_flags(int area, int oredFlags)
{
	CONTINUE_IF_ELSE_V(! mNavMeshType)

	//add area with corresponding ored ability flags
	mPolyAreaFlags[area] = oredFlags;
}

/**
 * Returns the area's 'ored' flags, or a negative number on error.
 */
INLINE int RNNavMesh::get_area_flags(int area) const
{
	map<int, int>::const_iterator iter = mPolyAreaFlags.find(area);
	return iter != mPolyAreaFlags.end() ? (*iter).second : RN_ERROR;
}

/**
 * Returns the area's traversal cost, or a negative number on error.
 */
INLINE float RNNavMesh::get_crowd_area_cost(int area) const
{
	map<int, float>::const_iterator iter = mPolyAreaCost.find(area);
	return iter != mPolyAreaCost.end() ? (*iter).second : RN_ERROR;
}

/**
 * Returns the RNCrowdAgent include flags.
 */
INLINE int RNNavMesh::get_crowd_include_flags() const
{
	return mCrowdIncludeFlags;
}

/**
 * Returns the RNCrowdAgent exclude flags.
 */
INLINE int RNNavMesh::get_crowd_exclude_flags() const
{
	return mCrowdExcludeFlags;
}

/**
 * Returns the convex volume's unique reference (>0) given its
 * index into the list of defined convex volumes.
 */
INLINE int RNNavMesh::get_convex_volume(int index) const
{
	return mConvexVolumes[index].get_second().get_ref();
}

/**
 * Returns the number of convex volumes defined for this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_convex_volumes() const
{
	return (int)mConvexVolumes.size();
}

/**
 * Returns the off mesh connection's unique reference (>0) given its
 * index into the list of defined off mesh connections.
 */
INLINE int RNNavMesh::get_off_mesh_connection(int index) const
{
	return mOffMeshConnections[index].get_second().get_ref();
}

/**
 * Returns the number of off mesh connections defined for this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_off_mesh_connections() const
{
	return (int)mOffMeshConnections.size();
}

/**
 * Gets an obstacle's unique reference (>0) given its index into the
 * list of defined obstacles.
 */
INLINE int RNNavMesh::get_obstacle(int index) const
{
	CONTINUE_IF_ELSE_R(mNavMeshTypeEnum == OBSTACLE, RN_ERROR)

	return (int) mObstacles[index].get_first().get_ref();
}

/**
 * Returns the number of obstacles added to this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_obstacles() const
{
	CONTINUE_IF_ELSE_R(mNavMeshTypeEnum == OBSTACLE, RN_ERROR)

	return (int)mObstacles.size();
}

/**
 * Returns the underlying NavMeshType tile settings (only TILE and OBSTACLE).
 */
INLINE RNNavMeshTileSettings RNNavMesh::get_nav_mesh_tile_settings() const
{
	return mNavMeshTileSettings;
}

/**
 * Returns the RNCrowdAgent given its index.
 */
INLINE PT(RNCrowdAgent) RNNavMesh::get_crowd_agent(int index) const
{
	CONTINUE_IF_ELSE_R((index >= 0) && (index < (int)mCrowdAgents.size()), NULL)

	return mCrowdAgents[index];
}

/**
 * Returns the number of RNCrowdAgents added to this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_crowd_agents() const
{
	return (int)mCrowdAgents.size();
}

/**
 * Returns the RNCrowdAgent given its index.
 */
INLINE PT(RNCrowdAgent) RNNavMesh::operator [](int index) const
{
	return get_crowd_agent(index);
}

/**
 * Returns the number of RNCrowdAgents added to this RNNavMesh.
 */
INLINE int RNNavMesh::size() const
{
	return get_num_crowd_agents();
}

/**
 * Resets RNNavMesh.
 */
inline void RNNavMesh::do_reset()
{
	//
	mOwnerObject.clear();
	mNavMeshType = NULL;
	mNavMeshTypeEnum = SOLO;
	mGeom = NULL;
	mCtx = NULL;
	mMeshName.clear();
	mReferenceNP.clear();
	mReferenceDebugNP.clear();
	mNavMeshSettings = RNNavMeshSettings();
	mNavMeshTileSettings = RNNavMeshTileSettings();
	mPolyAreaFlags.clear();
	mPolyAreaCost.clear();
	mCrowdIncludeFlags = mCrowdExcludeFlags = 0;
	mConvexVolumes.clear();
	mOffMeshConnections.clear();
	mObstacles.clear();
	mCrowdAgents.clear();
	mRef = 0;
#ifdef RN_DEBUG
	mDebugNodePath.clear();
	mDebugCamera.clear();
	mDD = NULL;
	mDDM = NULL;
	mEnableDrawUpdate = false;
	mDDUnsetup = NULL;
#endif //RN_DEBUG
}

/**
 * Returns the underlying rnsup::InputGeom.
 */
inline rnsup::InputGeom* RNNavMesh::get_recast_input_geom() const
{
	return mGeom;
}

/**
 * Returns the underlying dtNavMesh.
 */
inline dtNavMesh* RNNavMesh::get_recast_nav_mesh() const
{
	return (mNavMeshType ? mNavMeshType->getNavMesh() : NULL);
}

/**
 * Returns the underlying dtNavMeshQuery.
 */
inline dtNavMeshQuery* RNNavMesh::get_recast_nav_mesh_query() const
{
	return (mNavMeshType ? mNavMeshType->getNavMeshQuery() : NULL);
}

/**
 * Returns the underlying dtCrowd.
 */
inline dtCrowd* RNNavMesh::get_recast_crowd() const
{
	return (mNavMeshType ? mNavMeshType->getCrowd() : NULL);
}

/**
 * Returns a reference to underlying NavMeshType.
 */
inline rnsup::NavMeshType& RNNavMesh::get_nav_mesh_type() const
{
	return *mNavMeshType;
}

/**
 *
 */
inline int RNNavMesh::unique_ref()
{
	return ++mRef;
}

/**
 * NavMeshType reference conversion function.
 */
inline RNNavMesh::operator rnsup::NavMeshType&()
{
	return *mNavMeshType;
}

INLINE ostream &operator <<(ostream &out, const RNNavMesh& navMesh)
{
	navMesh.output(out);
	return out;
}

#endif /* RNNAVMESH_I_ */
