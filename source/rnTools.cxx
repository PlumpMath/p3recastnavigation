/**
 * \file rnTools.cxx
 *
 * \date 2016-03-18
 * \author consultit
 */

#include "../rnTools.h"

pvector<string> parseCompoundString(
		const string& srcCompoundString, char separator)
{
	//erase blanks
	string compoundString = srcCompoundString;
	compoundString = eraseCharacter(compoundString, ' ');
	compoundString = eraseCharacter(compoundString, '\t');
	compoundString = eraseCharacter(compoundString, '\n');
	//parse
	pvector<string> substrings;
	int len = compoundString.size() + 1;
	char* dest = new char[len];
	strncpy(dest, compoundString.c_str(), len);
	//find
	char* pch;
	char* start = dest;
	bool stop = false;
	while (not stop)
	{
		string substring("");
		pch = strchr(start, separator);
		if (pch != NULL)
		{
			//insert the substring
			substring.append(start, pch - start);
			start = pch + 1;
			substrings.push_back(substring);
		}
		else
		{
			if (start < &dest[len - 1])
			{
				//insert the last not empty substring
				substring.append(start, &dest[len - 1] - start);
				substrings.push_back(substring);
			}
			else if (start == &dest[len - 1])
			{
				//insert the last empty substring
				substrings.push_back(substring);
			}
			stop = true;
		}
	}
	delete[] dest;
	//
	return substrings;
}

string eraseCharacter(const string& source, int character)
{
	int len = source.size() + 1;
	char* dest = new char[len];
	char* start = dest;
	strncpy(dest, source.c_str(), len);
	//erase
	char* pch;
	pch = strchr(dest, character);
	while (pch != NULL)
	{
		len -= pch - start;
		memmove(pch, pch + 1, len - 1);
		start = pch;
		//continue
		pch = strchr(pch, character);
	}
	string outStr(dest);
	delete[] dest;
	return outStr;
}

string replaceCharacter(const string& source, int character,
		int replacement)
{
	int len = source.size() + 1;
	char* dest = new char[len];
	strncpy(dest, source.c_str(), len);
	//replace hyphens
	char* pch;
	pch = strchr(dest, character);
	while (pch != NULL)
	{
		*pch = replacement;
		pch = strchr(pch + 1, character);
	}
	string outStr(dest);
	delete[] dest;
	return outStr;
}

///ThrowEventData
void ThrowEventData::write_datagram(Datagram &dg) const
{
	dg.add_bool(mEnable);
	dg.add_string(mEventName);
	dg.add_bool(mThrown);
	dg.add_stdfloat(mTimeElapsed);
	dg.add_uint32(mCount);
	dg.add_stdfloat(mFrequency);
	dg.add_stdfloat(mPeriod);
}

void ThrowEventData::read_datagram(DatagramIterator &scan)
{
	mEnable = scan.get_bool();
	mEventName = scan.get_string();
	mThrown = scan.get_bool();
	mTimeElapsed = scan.get_stdfloat();
	mCount = scan.get_uint32();
	mFrequency = scan.get_stdfloat();
	mPeriod = scan.get_stdfloat();
}

///NavMeshSettings
/**
 *
 */
RNNavMeshSettings::RNNavMeshSettings()
{
}
/**
 * Writes the NavMeshSettings into a datagram.
 */
void RNNavMeshSettings::write_datagram(Datagram &dg) const
{
	dg.add_stdfloat(get_cellSize());
	dg.add_stdfloat(get_cellHeight());
	dg.add_stdfloat(get_agentHeight());
	dg.add_stdfloat(get_agentRadius());
	dg.add_stdfloat(get_agentMaxClimb());
	dg.add_stdfloat(get_agentMaxSlope());
	dg.add_stdfloat(get_regionMinSize());
	dg.add_stdfloat(get_regionMergeSize());
	dg.add_stdfloat(get_edgeMaxLen());
	dg.add_stdfloat(get_edgeMaxError());
	dg.add_stdfloat(get_vertsPerPoly());
	dg.add_stdfloat(get_detailSampleDist());
	dg.add_stdfloat(get_detailSampleMaxError());
	dg.add_int32(get_partitionType());
}

/**
 * Restores the NavMeshSettings from the datagram.
 */
void RNNavMeshSettings::read_datagram(DatagramIterator &scan)
{
	set_cellSize(scan.get_stdfloat());
	set_cellHeight(scan.get_stdfloat());
	set_agentHeight(scan.get_stdfloat());
	set_agentRadius(scan.get_stdfloat());
	set_agentMaxClimb(scan.get_stdfloat());
	set_agentMaxSlope(scan.get_stdfloat());
	set_regionMinSize(scan.get_stdfloat());
	set_regionMergeSize(scan.get_stdfloat());
	set_edgeMaxLen(scan.get_stdfloat());
	set_edgeMaxError(scan.get_stdfloat());
	set_vertsPerPoly(scan.get_stdfloat());
	set_detailSampleDist(scan.get_stdfloat());
	set_detailSampleMaxError(scan.get_stdfloat());
	set_partitionType(scan.get_int32());
}

///NavMeshTileSettings
/**
 *
 */
RNNavMeshTileSettings::RNNavMeshTileSettings()
{
}
/**
 * Writes the NavMeshTileSettings into a datagram.
 */
void RNNavMeshTileSettings::write_datagram(Datagram &dg) const
{
	dg.add_stdfloat(get_buildAllTiles());
	dg.add_int32(get_maxTiles());
	dg.add_int32(get_maxPolysPerTile());
	dg.add_stdfloat(get_tileSize());
}
/**
 * Restores the NavMeshTileSettings from the datagram.
 */
void RNNavMeshTileSettings::read_datagram(DatagramIterator &scan)
{
	set_buildAllTiles(scan.get_stdfloat());
	set_maxTiles(scan.get_int32());
	set_maxPolysPerTile(scan.get_int32());
	set_tileSize(scan.get_stdfloat());
}

///CrowdAgentParams
/**
 *
 */
RNCrowdAgentParams::RNCrowdAgentParams()
{
}

/**
 * Writes the CrowdAgentParams into a datagram.
 */
void RNCrowdAgentParams::write_datagram(Datagram &dg) const
{
	dg.add_stdfloat(get_radius());
	dg.add_stdfloat(get_height());
	dg.add_stdfloat(get_maxAcceleration());
	dg.add_stdfloat(get_maxSpeed());
	dg.add_stdfloat(get_collisionQueryRange());
	dg.add_stdfloat(get_pathOptimizationRange());
	dg.add_stdfloat(get_separationWeight());
	dg.add_uint8(get_updateFlags());
	dg.add_uint8(get_obstacleAvoidanceType());
	dg.add_uint8(get_queryFilterType());
	//Note: void *dtCrowdAgentParams::userData is not used
}
/**
 * Restores the CrowdAgentParams from the datagram.
 */
void RNCrowdAgentParams::read_datagram(DatagramIterator &scan)
{
	set_radius(scan.get_stdfloat());
	set_height(scan.get_stdfloat());
	set_maxAcceleration(scan.get_stdfloat());
	set_maxSpeed(scan.get_stdfloat());
	set_collisionQueryRange(scan.get_stdfloat());
	set_pathOptimizationRange(scan.get_stdfloat());
	set_separationWeight(scan.get_stdfloat());
	set_updateFlags(scan.get_uint8());
	set_obstacleAvoidanceType(scan.get_uint8());
	set_queryFilterType(scan.get_uint8());
	//Note: void *dtCrowdAgentParams::userData is not used
}

///ValueList template
// Tell GCC that we'll take care of the instantiation explicitly here.
#ifdef __GNUC__
#pragma implementation
#endif

template class ValueList<string>;
template class ValueList<LPoint3f>;
template struct Pair<bool,float>;
