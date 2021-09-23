#pragma once

#include "types.h"

namespace roto
{
/*
 * Small test scene with 3 cameras and 3 tracks:
 *            *cam2
 *      *cam0 |     *cam1
 *      |     v     |
 *      v           v
 *            *track2
 *      *track0     *track1
 *  z  y
 *  |/_x
 */
template<typename RotationType>
std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupSmallTestScene();

/*
 * Big test scene with 10x10 cameras and a point cloud with 1000 tracks:
 *                   *cam90       ...      *cam99
 *               ... |                  ...|
 *            *cam10 v              ...    v
 *      *cam0 |     *cam1  .... *cam9
 *      |     v     |           |
 *      v           v           v
 *
 *         *************************
 *    *********** Pointcloud ****************
 *         *************************
 *  z  y
 *  |/_x
 */
template<typename RotationType>
std::pair<MeasuredScene<RotationType>, OptimizedScene<RotationType>> setupBigTestScene();
} // namespace roto
