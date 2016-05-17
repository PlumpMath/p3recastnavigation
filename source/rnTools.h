/**
 * \file rnTools.h
 *
 * \date 2016-03-18
 * \author consultit
 */

#ifndef RNTOOLS_H_
#define RNTOOLS_H_

#ifdef _WIN32
#include <ciso646>
#define STRTOF (float)strtod
#else
#define STRTOF strtof
#endif

#include "recastnavigation_includes.h"
#include "genericAsyncTask.h"
#include "lpoint3.h"

//
#ifndef CPPPARSER
#include "support/NavMeshType.h"
#endif //CPPPARSER

/**
 * \brief An automatic Singleton Utility.
 *
 * \note This Singleton class is based on the article "An automatic
 * Singleton Utility" by Scott Bilas in "Game Programming Gems 1" book.
 * Non multi-threaded.
 */
template<typename T> class Singleton
{
	static T* ms_Singleton;

public:
	Singleton(void)
	{
		assert(!ms_Singleton);
		unsigned long int offset = (unsigned long int) (T*) 1
				- (unsigned long int) (Singleton<T>*) (T*) 1;
		ms_Singleton = (T*) ((unsigned long int) this + offset);
	}
	~Singleton(void)
	{
		assert(ms_Singleton);
		ms_Singleton = 0;
	}
	static T& GetSingleton(void)
	{
		assert(ms_Singleton);
		return (*ms_Singleton);
	}
	static T* GetSingletonPtr(void)
	{
		return (ms_Singleton);
	}
};

template<typename T> T* Singleton<T>::ms_Singleton = 0;

/**
 * \brief A std::pair wrapper
 */
template<typename T1, typename T2> struct Pair
{
PUBLISHED:
	Pair() :
			mPair()
	{
	}
	Pair(const T1& first, const T2& second) :
			mPair(first, second)
	{
	}
	bool operator== (const Pair &other) const
	{
		return mPair == other.mPair;
	}
	INLINE void set_first(const T1& first)
	{
		mPair.first = first;
	}
	INLINE T1 get_first() const
	{
		return mPair.first;
	}
	INLINE void set_second(const T2& second)
	{
		mPair.second = second;
	}
	INLINE T2 get_second() const
	{
		return mPair.second;
	}

public:
	T1& first()
	{
		return mPair.first;
	}
	T2& second()
	{
		return mPair.second;
	}
private:
	pair<T1, T2> mPair;
};

/**
 * \brief A pair that can be used with PT/CPT (C++ only)
 */
template<typename T1, typename T2> struct PairRC: public Pair<T1, T2>,
		public ReferenceCount
{
public:
	PairRC() :
			Pair<T1, T2>()
	{
	}
	PairRC(const T1& first, const T2& second) :
			Pair<T1, T2>(first, second)
	{
	}
};

/**
 * \brief Template struct for generic Task Function interface
 *
 * The effective Tasks are composed by a Pair of an object and
 * a method (member function) doing the effective task.
 * To register a task:
 * 1) in class A define a (pointer to) TaskData member:
 * \code
 * 	SMARTPTR(TaskInterface<A>::TaskData) myData;
 * \endcode
 * 2) and a method (that will execute the real task) with signature:
 * \code
 * 	AsyncTask::DoneStatus myTask(GenericAsyncTask* task);
 * \endcode
 * 3) in code associate to myData a new TaskData referring to this
 * class instance and myTask, then create a new GenericAsyncTask
 * referring to taskFunction and with data parameter equal to
 * myData (reinterpreted as void*):
 * \code
 * 	myData = new TaskInterface<A>::TaskData(this, &A::myTask);
 * 	AsyncTask* task = new GenericAsyncTask("my task",
 * 							&TaskInterface<A>::taskFunction,
 * 							reinterpret_cast<void*>(myData.p()));
 * \endcode
 * 4) finally register the async-task to your manager:
 * \code
 * 	pandaFramework.get_task_mgr().add(task);
 * 	\endcode
 * From now on myTask will execute the task, while being able
 * to refer directly to data members of the A class instance.
 */
template<typename A> struct TaskInterface
{
	typedef AsyncTask::DoneStatus (A::*TaskPtr)(GenericAsyncTask* taskFunction);
	typedef PairRC<A*, TaskPtr> TaskData;
	static AsyncTask::DoneStatus taskFunction(GenericAsyncTask* task,
			void * data)
	{
		TaskData* appData = reinterpret_cast<TaskData*>(data);
		return ((appData->first())->*(appData->second()))(task);
	}
};

/**
 * \brief Throwing event data.
 *
 * Data related to throwing events by components.
 */
class ThrowEventData
{
	struct Period
	{
		Period& operator =(float value)
		{
			mPeriod = value;
			return *this;
		}
		operator float() const
		{
			return mPeriod;
		}
	private:
		float mPeriod;
	};
	struct Frequency
	{
		Frequency(float value) :
				mFrequency(value >= 0 ? value: -value)
		{
			mFrequency <= FLT_MIN ?
					mPeriod = FLT_MAX : mPeriod = 1.0 / mFrequency;
			mEventData = NULL;
		}
		Frequency& operator =(float value)
		{
			value >= 0 ? mFrequency = value : mFrequency = -value;
			mFrequency <= FLT_MIN ?
					mPeriod = FLT_MAX : mPeriod = 1.0 / mFrequency;
			if (mEventData != NULL)
			{
				mEventData->mPeriod = mPeriod;
			}
			return *this;
		}
		operator float() const
		{
			return mFrequency;
		}
		void setEventData(ThrowEventData *eventData)
		{
			mEventData = eventData;
		}
	private:
		float mFrequency;
		Period mPeriod;
		ThrowEventData *mEventData;
	};

public:
	ThrowEventData() :
			mEnable(false), mEventName(string("")), mThrown(false), mTimeElapsed(
					0), mCount(0), mFrequency(30.0)
	{
		mFrequency.setEventData(this);
	}
	bool mEnable;
	string mEventName;
	bool mThrown;
	float mTimeElapsed;
	unsigned int mCount;
	Frequency mFrequency;
	Period mPeriod;

public:
	void write_datagram(Datagram &dg) const;
	void read_datagram(DatagramIterator &scan);
};

/**
 * \brief Declarations for parameters management.
 */
typedef multimap<string, string> ParameterTable;
typedef multimap<string, string>::iterator ParameterTableIter;
typedef multimap<string, string>::const_iterator ParameterTableConstIter;
typedef map<string, ParameterTable> ParameterTableMap;
typedef pair<string, string> ParameterNameValue;

/**
 * Template function for conversion values to string.
 */
template<typename Type> string str(Type value)
{
	return static_cast<ostringstream&>(ostringstream().operator <<(value)).str();
}

/**
 * \brief Parses a string composed by substrings separated by a character
 * separator.\n
 * \note all blanks are erased before parsing.
 * @param compoundString The source string.
 * @param separator The character separator.
 * @return The substrings vector.
 */
pvector<string> parseCompoundString(
		const string& srcCompoundString, char separator);

/**
 * \brief Into a given string, replaces any occurrence of a character with
 * another character.
 * @param source The source string.
 * @param character To be replaced character.
 * @param replacement Replaced character.
 * @return The result string.
 */
string replaceCharacter(const string& source, int character,
		int replacement);

/**
 * \brief Into a given string, erases any occurrence of a given character.
 * @param source The source string.
 * @param character To be erased character.
 * @return The result string.
 */
string eraseCharacter(const string& source, int character);

///NavMesh settings.
struct EXPORT_CLASS RNNavMeshSettings
{
PUBLISHED:
	RNNavMeshSettings();
#ifndef CPPPARSER
	RNNavMeshSettings(const rnsup::NavMeshSettings& navMeshSettings) :
			_navMeshSettings(navMeshSettings)
	{
	}
	operator rnsup::NavMeshSettings() const
	{
		return _navMeshSettings;
	}
#endif
	INLINE float get_cellSize() const;
	INLINE void set_cellSize(float value);
	INLINE float get_cellHeight() const;
	INLINE void set_cellHeight(float value);
	INLINE float get_agentHeight() const;
	INLINE void set_agentHeight(float value);
	INLINE float get_agentRadius() const;
	INLINE void set_agentRadius(float value);
	INLINE float get_agentMaxClimb() const;
	INLINE void set_agentMaxClimb(float value);
	INLINE float get_agentMaxSlope() const;
	INLINE void set_agentMaxSlope(float value);
	INLINE float get_regionMinSize() const;
	INLINE void set_regionMinSize(float value);
	INLINE float get_regionMergeSize() const;
	INLINE void set_regionMergeSize(float value);
	INLINE float get_edgeMaxLen() const;
	INLINE void set_edgeMaxLen(float value);
	INLINE float get_edgeMaxError() const;
	INLINE void set_edgeMaxError(float value);
	INLINE float get_vertsPerPoly() const;
	INLINE void set_vertsPerPoly(float value);
	INLINE float get_detailSampleDist() const;
	INLINE void set_detailSampleDist(float value);
	INLINE float get_detailSampleMaxError() const;
	INLINE void set_detailSampleMaxError(float value);
	INLINE int get_partitionType() const;
	INLINE void set_partitionType(int value);
private:
	rnsup::NavMeshSettings _navMeshSettings;

public:
	void write_datagram(Datagram &dg) const;
	void read_datagram(DatagramIterator &scan);
};

///NavMesh tile settings.
struct EXPORT_CLASS RNNavMeshTileSettings
{
PUBLISHED:
	RNNavMeshTileSettings();
#ifndef CPPPARSER
	RNNavMeshTileSettings(const rnsup::NavMeshTileSettings& navMeshTileSettings) :
			_navMeshTileSettings(navMeshTileSettings)
	{
	}
	operator rnsup::NavMeshTileSettings() const
	{
		return _navMeshTileSettings;
	}
#endif //CPPPARSER
	INLINE bool get_buildAllTiles() const;
	INLINE void set_buildAllTiles(bool value);
	INLINE int get_maxTiles() const;
	INLINE void set_maxTiles(int value);
	INLINE int get_maxPolysPerTile() const;
	INLINE void set_maxPolysPerTile(int value);
	INLINE float get_tileSize() const;
	INLINE void set_tileSize(float value);
private:
	rnsup::NavMeshTileSettings _navMeshTileSettings;

public:
	void write_datagram(Datagram &dg) const;
	void read_datagram(DatagramIterator &scan);
};

///CrowdAgentParams
struct EXPORT_CLASS RNCrowdAgentParams
{
PUBLISHED:
	RNCrowdAgentParams();
#ifndef CPPPARSER
	RNCrowdAgentParams(const dtCrowdAgentParams& crowdAgentParams) :
			_dtCrowdAgentParams(crowdAgentParams)
	{
	}
	operator dtCrowdAgentParams() const
	{
		return _dtCrowdAgentParams;
	}
#endif //CPPPARSER
	INLINE float get_radius() const;
	INLINE void set_radius(float value);
	INLINE float get_height() const;
	INLINE void set_height(float value);
	INLINE float get_maxAcceleration() const;
	INLINE void set_maxAcceleration(float value);
	INLINE float get_maxSpeed() const;
	INLINE void set_maxSpeed(float value);
	INLINE float get_collisionQueryRange() const;
	INLINE void set_collisionQueryRange(float value);
	INLINE float get_pathOptimizationRange() const;
	INLINE void set_pathOptimizationRange(float value);
	INLINE float get_separationWeight() const;
	INLINE void set_separationWeight(float value);
	INLINE unsigned char get_updateFlags() const;
	INLINE void set_updateFlags(unsigned char value);
	INLINE unsigned char get_obstacleAvoidanceType() const;
	INLINE void set_obstacleAvoidanceType(unsigned char value);
	INLINE unsigned char get_queryFilterType() const;
	INLINE void set_queryFilterType(unsigned char value);
	INLINE void* get_userData() const;
	INLINE void set_userData(void* value);

private:
	dtCrowdAgentParams _dtCrowdAgentParams;

public:
	void write_datagram(Datagram &dg) const;
	void read_datagram(DatagramIterator &scan);
};

///ValueList template
template<typename Type>
class ValueList
{
PUBLISHED:
	ValueList(unsigned int size=0);
	ValueList(const ValueList &copy);
	ValueList(ValueList&& copy);
	INLINE ~ValueList();

	INLINE void operator =(const ValueList &copy);
	INLINE void operator =(ValueList&& copy);
	INLINE bool operator== (const ValueList &other) const;
	INLINE void add_value(const Type& value);
	bool remove_value(const Type& value);
	bool has_value(const Type& value) const;
	void add_values_from(const ValueList &other);
	void remove_values_from(const ValueList &other);
	INLINE void clear();
	INLINE int get_num_values() const;
	INLINE Type get_value(int index) const;
	MAKE_SEQ(get_values, get_num_values, get_value);
	INLINE Type operator [](int index) const;
	INLINE int size() const;
	INLINE void operator +=(const ValueList &other);
	INLINE ValueList operator +(const ValueList &other) const;

#ifndef CPPPARSER
public:
	operator plist<Type>() const;
	operator pvector<Type>() const;
#endif //CPPPARSER

private:
	pvector<Type> _values;
};

///Result values
#define RN_SUCCESS 0
#define RN_ERROR -1
#define RN_NAVMESH_NULL -2

///inline
#include "rnTools.I"

#if !defined(CPPPARSER) && !defined(_WIN32)
extern template class ValueList<string>;
extern template class ValueList<LPoint3f>;
extern template struct Pair<bool,float>;
#endif //CPPPARSER

#endif /* RNTOOLS_H_ */
