#pragma once

#include "indexer/feature_data.hpp"
#include "indexer/ftypes_matcher.hpp"
#include "indexer/indoor_level.hpp"

#include "geometry/point2d.hpp"
#include "geometry/rect2d.hpp"

#include <string_view>
#include <vector>

namespace df
{
// Returns true if a feature isn't part of the active indoor level and should be skipped at rendering
//
// Indoor-tagged polygons (indoor=*) are always filtered by the active level.
// Features with a level=* tag are only sometimes filtered (see below)
// We intentionally don't filter building outlines/parts or non-level-tagged features.
//
// When no indoor level is active (kNoActiveLevel) nothing is skipped.
inline bool ShouldSkipIndoorFeature(feature::TypesHolder const & types, std::string_view levelMeta,
                                    double activeLevel, m2::PointD const & featureCenter,
                                    std::vector<m2::RectD> const & indoorPolygonRects,
                                    std::vector<double> const & availableLevels)
{
  if (!indoor::HasActiveLevel(activeLevel))
    return false;

  bool const isIndoor = ftypes::IsIndoorChecker::Instance()(types);
  bool const isLeveled = !levelMeta.empty() && !ftypes::IsBuildingChecker::Instance()(types) &&
                         !ftypes::IsBuildingPartChecker::Instance()(types);
  if (!isIndoor && !isLeveled)
    return false;  

  // POIs with level=* etc
  if (!isIndoor)
  {
    // only filter if the feature center is within ~5m of an indoor=* polygon.
    double constexpr kProximityDeg = 0.00005;
    if (!indoorPolygonRects.empty())
    {
      bool nearIndoor = false;
      for (auto const & r : indoorPolygonRects)
      {
        m2::RectD expanded(r.minX() - kProximityDeg, r.minY() - kProximityDeg,
                           r.maxX() + kProximityDeg, r.maxY() + kProximityDeg);
        if (expanded.IsPointInside(featureCenter))
        {
          nearIndoor = true;
          break;
        }
      }
      if (!nearIndoor)
        return false;
    }

    // If a POI has some other level like -0.5 that isn't part of our indoor=* level picker, don't filter it
    bool const levelIsInBuilding = std::any_of(availableLevels.begin(), availableLevels.end(),
      [&](double buildingLevel) { return indoor::LevelsContain(levelMeta, buildingLevel); });
    if (!levelIsInBuilding)
      return false;
  }

  // It's an indoor element. Don't filter when it's level(s) are active
  return !indoor::LevelsContain(levelMeta, activeLevel);
}
}  // namespace df
