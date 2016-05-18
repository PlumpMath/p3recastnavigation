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
 * Sets the owner object's NodePath this RNNavMesh is attached to.
 */
INLINE void RNNavMesh::set_owner_node_path(const NodePath& ownerObject)
{
	mOwnerObject = ownerObject;
}

/**
 * Gets the owner object's NodePath this RNNavMesh is attached to.
 */
INLINE NodePath RNNavMesh::get_owner_node_path()const
{
	return mOwnerObject;
}

/**
 * Gets the owner object's input geometry min bounds.
 */
INLINE LVecBase3f RNNavMesh::get_recast_bounds_min() const
{
	return (mGeom ?
			rnsup::RecastToLVecBase3f(mGeom->getMeshBoundsMin()) :
			LVecBase3f::zero());
}

/**
 * Gets the owner object's input geometry max bounds.
 */
INLINE LVecBase3f RNNavMesh::get_recast_bounds_max() const
{
	return (mGeom ?
			rnsup::RecastToLVecBase3f(mGeom->getMeshBoundsMax()) :
			LVecBase3f::zero());
}

/**
 * Gets the underlying NavMeshType type: SOLO, TILE, OBSTACLES.
 */
INLINE RNNavMesh::RNNavMeshTypeEnum RNNavMesh::get_nav_mesh_type_enum() const
{
	return mNavMeshTypeEnum;
}

/**
 * Gets the underlying NavMeshType settings.
 */
INLINE RNNavMeshSettings RNNavMesh::get_nav_mesh_settings() const
{
	return mNavMeshSettings;
}

/**
 * Sets the Recast area's ored flags.
 * Can be set only before RNNavMesh is setup.
 * \note oredFlags: by default flags as specified in RNNavMeshPolyFlagsEnum.
 */
INLINE void RNNavMesh::set_area_flags(int area, int oredFlags)
{
	nassertv_always(! mNavMeshType)

	//add area with corresponding ored ability flags
	mPolyAreaFlags[area] = oredFlags;
}

/**
 * Returns the Recast area's ored flags.
 */
INLINE int RNNavMesh::get_area_flags(int area)
{
	return mPolyAreaFlags[area];
}

/**
 * Returns the Recast area's traversal cost.
 */
INLINE float RNNavMesh::get_area_cost(int area)
{
	return mPolyAreaCost[area];
}

/**
 * Returns the Recast Crowd include flags.
 */
INLINE int RNNavMesh::get_crowd_include_flags()
{
	return mCrowdIncludeFlags;
}

/**
 * Returns the Recast Crowd exclude flags.
 */
INLINE int RNNavMesh::get_crowd_exclude_flags()
{
	return mCrowdExcludeFlags;
}

/**
 * Gets a convex volume point list by index.
 */
INLINE ValueList<LPoint3f> RNNavMesh::get_convex_volume(int index) const
{
	return mConvexVolumes[index].get_first();
}

/**
 * Returns the number of convex volumes defined on this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_convex_volumes() const
{
	return (int)mConvexVolumes.size();
}

/**
 * Gets an off mesh connection point pair (as 2 element list) by index.
 */
INLINE ValueList<LPoint3f> RNNavMesh::get_off_mesh_connection(int index) const
{
	return mOffMeshConnections[index].get_first();
}

/**
 * Returns the number of off mesh connections defined for this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_off_mesh_connections() const
{
	return (int)mOffMeshConnections.size();
}

/**
 * Gets an obstacle's unique reference (>0) by index (i.e. position into internal repository).
 */
INLINE int RNNavMesh::get_obstacle(int index) const
{
	nassertr_always(mNavMeshTypeEnum == OBSTACLE, RN_ERROR)

	return (int) mObstacles[index].get_first();
}

/**
 * Returns the number of obstacles placed on this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_obstacles() const
{
	nassertr_always(mNavMeshTypeEnum == OBSTACLE, RN_ERROR)

	return (int)mObstacles.size();
}

/**
 * Sets up the underlying NavMeshType tile settings (only TILE and OBSTACLE).
 */
INLINE RNNavMeshTileSettings RNNavMesh::get_nav_mesh_tile_settings() const
{
	return mNavMeshTileSettings;
}

/**
 * Gets a RNCrowdAgent by index.
 */
INLINE PT(RNCrowdAgent) RNNavMesh::get_crowd_agent(int index) const
{
	nassertr_always((index >= 0) && (index < (int)mCrowdAgents.size()), NULL)

	return mCrowdAgents[index];
}

/**
 * Returns the number of RNCrowdAgents handled by this RNNavMesh.
 */
INLINE int RNNavMesh::get_num_crowd_agents() const
{
	return (int)mCrowdAgents.size();
}

/**
 * Gets a RNCrowdAgent by index.
 */
INLINE PT(RNCrowdAgent) RNNavMesh::operator [](int index) const
{
	return get_crowd_agent(index);
}

/**
 * Returns the number of RNCrowdAgents handled by this RNNavMesh.
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
	mNavMeshSettings = RNNavMeshSettings();
	mNavMeshTileSettings = RNNavMeshTileSettings();
	mPolyAreaFlags.clear();
	mPolyAreaCost.clear();
	mCrowdIncludeFlags = mCrowdExcludeFlags = 0;
	mConvexVolumes.clear();
	mOffMeshConnections.clear();
	mObstacles.clear();
	mCrowdAgents.clear();
#ifdef RN_DEBUG
	mDebugNodePath.clear();
	mDebugCamera.clear();
	mDD = NULL;
	mDDM = NULL;
	mEnableDrawUpdate = false;
#endif //RN_DEBUG
}

/**
 * Gets the underlying rnsup::InputGeom.
 */
inline rnsup::InputGeom* RNNavMesh::get_recast_input_geom() const
{
	return mGeom;
}

/**
 * Gets the underlying dtNavMesh.
 */
inline dtNavMesh* RNNavMesh::get_recast_nav_mesh() const
{
	return (mNavMeshType ? mNavMeshType->getNavMesh() : NULL);
}

/**
 * Gets the underlying dtNavMeshQuery.
 */
inline dtNavMeshQuery* RNNavMesh::get_recast_nav_mesh_query() const
{
	return (mNavMeshType ? mNavMeshType->getNavMeshQuery() : NULL);
}

/**
 * Gets the underlying dtCrowd.
 */
inline dtCrowd* RNNavMesh::get_recast_crowd() const
{
	return (mNavMeshType ? mNavMeshType->getCrowd() : NULL);
}

/**
 * NavMeshType reference getter function.
 */
inline rnsup::NavMeshType& RNNavMesh::get_nav_mesh_type()
{
	return *mNavMeshType;
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
