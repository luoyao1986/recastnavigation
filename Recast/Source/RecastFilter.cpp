//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Recast.h"
#include "RecastAssert.h"

#include <stdlib.h>
#include <vector>
#include <list>

namespace
{
	const int MAX_HEIGHTFIELD_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
}

void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousAreaID = RC_NULL_AREA;

			// For each span in the column...
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;

				// If current span is not walkable, but there is walkable span just below it and the height difference
				// is small enough for the agent to walk over, mark the current span as walkable too.
				if (!walkable && previousWasWalkable && (int)span->smax - (int)previousSpan->smax <= walkableClimb)
				{
					span->area = previousAreaID;
				}

				// Copy the original walkable value regardless of whether we changed it.
				// This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
				previousWasWalkable = walkable;
				previousAreaID = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	
	// Mark spans that are adjacent to a ledge as unwalkable..
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non-walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;

				// The difference between this walkable area and the lowest neighbor walkable area.
				// This is the difference between the current span and all neighbor spans that have
				// enough space for an agent to move between, but not accounting at all for surface slope.
				int lowestNeighborFloorDifference = MAX_HEIGHTFIELD_HEIGHT;

				// Min and max height of accessible neighbours.
				int lowestTraversableNeighborFloor = span->smax;
				int highestTraversableNeighborFloor = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);

					// Skip neighbours which are out of bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
					{
						lowestNeighborFloorDifference = -walkableClimb - 1;
						break;
					}

					const rcSpan* neighborSpan = heightfield.spans[neighborX + neighborZ * xSize];

					// The most we can step down to the neighbor is the walkableClimb distance.
					// Start with the area under the neighbor span
					int neighborCeiling = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHTFIELD_HEIGHT;

					// Skip neighbour if the gap between the spans is too small.
					if (rcMin(ceiling, neighborCeiling) - floor >= walkableHeight)
					{
						lowestNeighborFloorDifference = (-walkableClimb - 1);
						break;
					}

					// For each span in the neighboring column...
					for (; neighborSpan != NULL; neighborSpan = neighborSpan->next)
					{
						const int neighborFloor = (int)neighborSpan->smax;
						neighborCeiling = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHTFIELD_HEIGHT;

						// Only consider neighboring areas that have enough overlap to be potentially traversable.
						if (rcMin(ceiling, neighborCeiling) - rcMax(floor, neighborFloor) < walkableHeight)
						{
							// No space to traverse between them.
							continue;
						}

						const int neighborFloorDifference = neighborFloor - floor;
						lowestNeighborFloorDifference = rcMin(lowestNeighborFloorDifference, neighborFloorDifference);

						// Find min/max accessible neighbor height.
						// Only consider neighbors that are at most walkableClimb away.
						if (rcAbs(neighborFloorDifference) <= walkableClimb)
						{
							// There is space to move to the neighbor cell and the slope isn't too much.
							lowestTraversableNeighborFloor = rcMin(lowestTraversableNeighborFloor, neighborFloor);
							highestTraversableNeighborFloor = rcMax(highestTraversableNeighborFloor, neighborFloor);
						}
						else if (neighborFloorDifference < -walkableClimb)
						{
							// We already know this will be considered a ledge span so we can early-out
							break;
						}
					}
				}

				// The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
				// That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
				// (If this is the case, then biggestNeighborStepDown will be negative, so compare against the negative walkableClimb as a means of checking
				// the magnitude of the delta)
				if (lowestNeighborFloorDifference < -walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
				else if (highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;
				if (ceiling - floor < walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void _EvaluateRuggedArea(std::list< std::vector<rcSpan*> > &ConnectedSpanListsInColume, float SlopeThreshold, int RuggedAreaID)
{
	// 根据路径链条,算出total difference,再算出平均avgSlope.
	// 问题1:是否需要算出累积difference,来判断是否为长坡?
	// 问题2:是直接标记此area为rugged,还是在这个span后插入一个新的ruggedID的新span?
	for (auto& ConnectedSpanList : ConnectedSpanListsInColume)
	{
		rcSpan* startSpan = ConnectedSpanList.front();
		if (startSpan->area != RC_WALKABLE_AREA)
			continue;
		int TotalDiff = 0;
		int TotalConnectedSpanCount = 0;
		for (int i = 1; i < ConnectedSpanList.size(); ++i)
		{
			const int CurrentFloor = (int)ConnectedSpanList[i - 1]->smax;
			const int NeiborFloor = (int)ConnectedSpanList[i]->smax;
			int difference = rcAbs(CurrentFloor - NeiborFloor);
			TotalDiff += difference;
			++TotalConnectedSpanCount;
		}
		if (TotalConnectedSpanCount > 0)
		{
			float avgSlope = (float)TotalDiff / (float)TotalConnectedSpanCount;
			if (avgSlope >= SlopeThreshold)
			{
				// 大于阈值,需要标记成崎岖
				for (int n = 0; n < ConnectedSpanList.size(); ++n)
				{
					ConnectedSpanList[n]->area = RuggedAreaID;
				}
			}
		}
	}
}

std::list< std::vector<rcSpan*> > _InitSpanListInColume(rcSpan* span)
{
	std::list< std::vector<rcSpan*> >ConnectedSpanListsInColume;
	for (auto* spanInColume = span; spanInColume != NULL; spanInColume = spanInColume->next)
	{
		std::vector<rcSpan*> newSpanList;
		newSpanList.push_back(spanInColume); // 提前加入起点
		ConnectedSpanListsInColume.push_back(newSpanList);
	}

	return ConnectedSpanListsInColume;
}

// 粗糙表面滤波并标记
// 某个cell,向某方向延展,可以通行,在n个相邻单位内,累积绝对坡度达到阈值SlopeThreshold,则此cell认为是崎岖,标记为RuggedAreaID
// 备注:X方向跑一次,Z方向跑一次.
void rcFilterRuggedAreaSpans(rcContext* context, const int walkableHeight, const int walkableClimb, float SlopeThreshold, int RuggedAreaID, rcHeightfield& heightfield)
{
	rcAssert(context);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Mark spans that are adjacent to a ledge as unwalkable..
	const int MAX_SLOPE_SAMPLE_STEP = 9;
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non-walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}
				//if (span->area == RuggedAreaID)
				//{
				//	continue;
				//}

				//x-axis
				// 初始化数据
				std::list< std::vector<rcSpan*> > ConnectedSpanListsInColume = _InitSpanListInColume(span);

				{
					// 两个迭代延伸点p,p+1. 下面步骤是为了找到所有能与OriginalSpan的X方向上能通行的点.形成一个路径链条.
					// 得到路径链条以后,再根据这个路径链条去统计每条路径链条上的total difference.
					// 然后根据这个total difference,来决定此span是否需要修改成崎岖area,(可能需要修改span链表,插入一个崎岖span的area)

					// 1.先计算出本点所有的能通行的路径链条.
					int CurrentSamplePointX = x;
					int NeighborSamplePointX = x + 1;
					for (int sampleStep = 0; sampleStep < MAX_SLOPE_SAMPLE_STEP; ++sampleStep)
					{
						if (CurrentSamplePointX < 0 || CurrentSamplePointX >= xSize || NeighborSamplePointX < 0 || NeighborSamplePointX >= xSize)
						{
							// 到头了
							break;
						}

						for (rcSpan* CurrentSpan = heightfield.spans[CurrentSamplePointX + z * xSize]; CurrentSpan != NULL; CurrentSpan = CurrentSpan->next)
						{
							if (CurrentSpan->area == RC_NULL_AREA)
							{
								// 遍历到不能走的点了,需要结束此Span的路径
								break;
							}
							const int CurrentFloor = (int)CurrentSpan->smax;
							for (rcSpan* NeighborSpan = heightfield.spans[NeighborSamplePointX + z * xSize]; NeighborSpan != NULL; NeighborSpan = NeighborSpan->next)
							{
								if (NeighborSpan->area == RC_NULL_AREA)
								{
									// 此NeighborSpan不让走,试试下一个
									continue;
								}
								const int NeiborFloor = (int)NeighborSpan->smax;
								int difference = rcAbs(CurrentFloor - NeiborFloor);
								if (difference <= walkableClimb)
								{
									for (auto& ConnectedSpanList : ConnectedSpanListsInColume)
									{
										// 如果list里的最后一个与current span相等,说明是本路径链条上的.(有可能有多条汇聚或交叉的情况)
										if (ConnectedSpanList.back() == CurrentSpan)
										{
											ConnectedSpanList.push_back(NeighborSpan);
											// 这里能break的原因是只需要找最下方能走的span,如果在一个climb内有多个可用的层,说明有很多小span很密,正常情况会被LowHeight给标记不可行走.
											// 所以应该只需要算最下层能走的情况吧?
											// 是否应该找最上一个可行走的区域?还需要观察一下实际测试情况.
											break;
										}
									}
								}
							}
						}
						++CurrentSamplePointX;
						++NeighborSamplePointX;
					}

					_EvaluateRuggedArea(ConnectedSpanListsInColume, SlopeThreshold, RuggedAreaID);
				}

				// z-axis
				// 重新初始化数据
				if (span->area != RuggedAreaID)
				{
					ConnectedSpanListsInColume = _InitSpanListInColume(span);
					// 两个迭代延伸点p,p+1. 下面步骤是为了找到所有能与OriginalSpan的X方向上能通行的点.形成一个路径链条.
					// 得到路径链条以后,再根据这个路径链条去统计每条路径链条上的total difference.
					// 然后根据这个total difference,来决定此span是否需要修改成崎岖area,(可能需要修改span链表,插入一个崎岖span的area)

					// 1.先计算出本点所有的能通行的路径链条.
					int CurrentSamplePointZ = z;
					int NeighborSamplePointZ = z + 1;
					for (int sampleStep = 0; sampleStep < MAX_SLOPE_SAMPLE_STEP; ++sampleStep)
					{
						if (CurrentSamplePointZ < 0 || CurrentSamplePointZ >= zSize || NeighborSamplePointZ < 0 || NeighborSamplePointZ >= zSize)
						{
							// 到头了
							break;
						}

						for (rcSpan* CurrentSpan = heightfield.spans[x + CurrentSamplePointZ * xSize]; CurrentSpan != NULL; CurrentSpan = CurrentSpan->next)
						{
							if (CurrentSpan->area == RC_NULL_AREA)
							{
								// 遍历到不能走的点了,需要结束此Span的路径
								break;
							}

							const int CurrentFloor = (int)CurrentSpan->smax;
							for (rcSpan* NeighborSpan = heightfield.spans[x + NeighborSamplePointZ * xSize]; NeighborSpan != NULL; NeighborSpan = NeighborSpan->next)
							{
								if (NeighborSpan->area == RC_NULL_AREA)
								{
									// 此NeighborSpan不让走,试试下一个
									continue;
								}

								const int NeiborFloor = (int)NeighborSpan->smax;
								int difference = rcAbs(CurrentFloor - NeiborFloor);
								if (difference <= walkableClimb)
								{
									for (auto& ConnectedSpanList : ConnectedSpanListsInColume)
									{
										// 如果list里的最后一个与current span相等,说明是本路径链条上的.(有可能有多条汇聚或交叉的情况)
										if (ConnectedSpanList.back() == CurrentSpan)
										{
											ConnectedSpanList.push_back(NeighborSpan);
											// 这里能break的原因是只需要找最下方能走的span,如果在一个climb内有多个可用的层,说明有很多小span很密,正常情况会被LowHeight给标记不可行走.
											// 所以应该只需要算最下层能走的情况吧?
											// 是否应该找最上一个可行走的区域?还需要观察一下实际测试情况.
											break;
										}
									}
								}
							}
						}
						++CurrentSamplePointZ;
						++NeighborSamplePointZ;
					}

					_EvaluateRuggedArea(ConnectedSpanListsInColume, SlopeThreshold, RuggedAreaID);
				}
			}
		}
	}
}