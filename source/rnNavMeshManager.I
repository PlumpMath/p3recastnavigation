/**
 * \file rnNavMeshManager.I
 *
 * \date 2016-03-29
 * \author consultit
 */

#ifndef RNNAVMESHMANAGER_I_
#define RNNAVMESHMANAGER_I_

///RNNavMeshManager inline definitions

/**
 * Returns the number of RNNavMeshes.
 */
INLINE int RNNavMeshManager::get_num_nav_meshes() const
{
	return (int) mNavMeshes.size();
}

/**
 * Returns the number of RNCrowdAgents.
 */
INLINE int RNNavMeshManager::get_num_crowd_agents() const
{
	return (int) mCrowdAgents.size();
}

/**
 * Get singleton pointer.
 */
INLINE RNNavMeshManager* RNNavMeshManager::get_global_ptr()
{
	return Singleton<RNNavMeshManager>::GetSingletonPtr();
}

/**
 * Get the collide mask.
 */
INLINE CollideMask RNNavMeshManager::get_collide_mask() const
{
	return mMask;
}

/**
 * Get the collision root.
 */
INLINE NodePath RNNavMeshManager::get_collision_root() const
{
	return mRoot;
}

/**
 * Get the collision traverser.
 */
INLINE CollisionTraverser* RNNavMeshManager::get_collision_traverser() const
{
	return mCTrav;
}

/**
 * Get the collision handler.
 */
INLINE CollisionHandlerQueue* RNNavMeshManager::get_collision_handler() const
{
	return mCollisionHandler;
}

/**
 * Get the collision ray.
 */
INLINE CollisionRay* RNNavMeshManager::get_collision_ray() const
{
	return mPickerRay;
}

#endif /* RNNAVMESHMANAGER_I_ */
