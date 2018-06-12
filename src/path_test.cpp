//
// Created by Liam O'Gorman on 6/2/18.
//

//
// Created by Liam O'Gorman on 5/28/18.
//

#include <limits.h>
#include <iostream>

#include "map_state.h"
#include "path.h"
#include "utils.h"

#include "gtest/gtest.h"

void ValidateDerivatives(const Path &path, double time_step) {
  std::vector<std::tuple<double, double, double>> path_derivatives =
      CalculateSpeedDerivatives(path, time_step);
  int count = 0;
  for (const auto &sample : path_derivatives) {
    double speed, accel, jerk;
    std::tie(speed, accel, jerk) = sample;
    EXPECT_LT(speed, mph_to_mps(50.0)) << ":s=" << speed;
    EXPECT_LT(accel, 6.1) << count << ":s2=" << accel;
    EXPECT_LT(jerk, 10.0) << count << "jerk=" << jerk;
    ++count;
  }
}

TEST(GeneratePaths, ValidFirstPath) {
  std::vector<double> x = {
      909.481199919322648, 909.484799677290425, 909.490799273903122,
      909.499198709159941, 909.509997983058838, 909.523197095595378,
      909.538796046761718, 909.556794836544782, 909.57719346492479,
      909.599991931873092, 909.625190237349329, 909.652788381299729,
      909.682786363653122, 909.715184184318559, 909.749981843181445,
      909.787179340099897, 909.826776674900316, 909.868773847373745,
      909.913170857270757, 909.95996770429656,  910.009164388106115,
      910.060760908298107, 910.114757264409604, 910.171153455909916,
      910.229949482194002, 910.291145342575874, 910.354741036281325,
      910.420736562440993, 910.489131920082173, 910.559927108121428,
      910.633122125355158, 910.708716970451974, 910.786711641942702,
      910.86710613821117,  910.949900457484432, 911.035094597821853,
      911.122688557104766, 911.212682333024986, 911.305075923072877,
      911.399869324525412, 911.497062534432871, 911.596655549605885,
      911.698648366601105, 911.803040981706545, 911.909833390926451,
      912.019025589965054, 912.130617574209964, 912.244609338714554,
      912.361000878179539, 912.479792186933537, 912.600983258912947,
      912.724574087640576, 912.850564666203468, 912.978954987229258,
      913.109745042861732, 913.242934824734562, 913.378524323944362,
      913.516513531021815, 913.656902435901316, 913.799691027889367,
      913.944879295631154, 914.092467227074621, 914.242454809433866,
      914.394842029149459, 914.549628871847176, 914.706815322294119,
      914.866401364353351, 915.028386980934101, 915.192772153941746,
      915.359556864222782, 915.52874109150855,  915.700324814354417,
      915.874308010076675, 916.050690654685582, 916.229472722814762,
      916.410654187646287, 916.594235020833139, 916.78021519241554,
      916.968594670734319, 917.159373422338376, 917.352551411888612,
      917.548128602055272, 917.746104953410963, 917.946480424316974,
      918.149254970804577, 918.354428546449867, 918.562001102241311,
      918.771972586441734, 918.984342944442005, 919.1991121186079,
      919.416280048119006, 919.635846668799672, 919.857811912940974,
      920.082175709114267, 920.308937981975419, 920.538098652059034,
      920.769657635563135, 921.003614844123149, 921.239970184574759,
      921.478723558705838};
  std::vector<double> y = {
      1128.67001391471399, 1128.67005565885779, 1128.67012523244944,
      1128.67022263555896, 1128.67034786836825, 1128.67050093125522,
      1128.67068182490152, 1128.67089055042538, 1128.67112710953597,
      1128.67139150471462, 1128.67168373941877, 1128.67200381830912,
      1128.67235174750203, 1128.67272753484576, 1128.67313119021969,
      1128.67356272585903, 1128.67402215670177, 1128.67450950076159,
      1128.67502477952303, 1128.6755680183619,  1128.67613924698867,
      1128.67673849991684, 1128.67736581695431, 1128.67802124371997,
      1128.67870483218258, 1128.67941664122554, 1128.6801567372338,
      1128.68092519470702, 1128.68172209689396, 1128.68254753645306,
      1128.68340161613651, 1128.68428444949723, 1128.68519616162143,
      1128.68613688988353, 1128.68710678472689, 1128.68810601046698,
      1128.68913474611941, 1128.69019318625101, 1128.69128154185637,
      1128.69240004125709, 1128.69354893102513, 1128.69472847693123,
      1128.69593896491597, 1128.69718070208523, 1128.69845401773068,
      1128.69975926437178, 1128.70109681882468, 1128.70246708329319,
      1128.70387048648399, 1128.7053074847463,  1128.70677856323482,
      1128.70828423709781, 1128.70982505268762, 1128.71140158879621,
      1128.71301445791414, 1128.71466430751366, 1128.71635182135606,
      1128.71807772082161, 1128.71984276626563, 1128.72164775839656,
      1128.72349353967866, 1128.72538099575831, 1128.72731105691537,
      1128.72928469953649, 1128.73130294761336, 1128.73336687426558,
      1128.73547760328552, 1128.73763631070892, 1128.73984422640797,
      1128.74210263570831, 1128.74441288103071, 1128.74677636355591,
      1128.74919454491328, 1128.75166894889253, 1128.75420116318105,
      1128.75679284112266, 1128.75944570350157, 1128.7621615403491,
      1128.76494221277517, 1128.76778965482185, 1128.77070587534149,
      1128.77369295989797, 1128.77675307269237, 1128.77988845850996,
      1128.78310144469265, 1128.78639444313467, 1128.78976995229959,
      1128.79323055926307, 1128.79677894177689, 1128.80041787035748,
      1128.80415021039721, 1128.80797892429723, 1128.81190707362612,
      1128.81593782129835, 1128.82007443377711, 1128.82432028330095,
      1128.82867885012956, 1128.83315372481593, 1128.83774861049824,
      1128.84246732521456};

  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  Path prev_path_map = PathFromVectors(x, y);
  ValidateDerivatives(prev_path_map, 1.0 / 50.0);
}

TEST(GeneratePaths, GenerateFirstReferencePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(
      prev_path_map, sdc_state, std::make_tuple(sdc_state.s(), sdc_state.d()),
      sdc_state.Lane(), map_state);
}

TEST(GeneratePaths, GenerateFirstDrivablePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(
      prev_path_map, sdc_state, std::make_tuple(sdc_state.s(), sdc_state.d()),
      sdc_state.Lane(), map_state);

  constexpr double time_step = 1.0 / 50.0;
  Path drivable_path = GenerateSDCPathByTimeSamples(
      map_state, ref_path, prev_path_map, sdc_state, 5.0, mph_to_mps(49.5),
      time_step, 2.0);

  ValidateDerivatives(drivable_path, time_step);
  const double dist = Distance(drivable_path.front(), drivable_path.back());
  GTEST_ASSERT_GE(dist, 0.0);
}

TEST(GeneratePaths, GenerateFirstPlan) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  Path prev_path_map;

  CarState state(909.48, 1128.67, 124.834, 6.16483, 0, 0);
  std::vector<double> x = {};
  std::vector<double> y = {};
  double end_path_s = 0;
  double end_path_d = 0;

  constexpr double time_step = 1.0 / 50.0;
  Plan plan =
      GeneratePathAndCost("Accel:1", prev_path_map, state, {}, map_state,
                          std::make_tuple(end_path_s, end_path_d), state.Lane(),
                          6.0, mph_to_mps(49.0), time_step, 2.0);
  DumpPathForUnitTest("plan", std::get<2>(plan));
  ValidateDerivatives(std::get<2>(plan), time_step);
}

TEST(GeneratePaths, GenerateReferencePathFromPrev) {
  CarState state(909.481, 1128.67, 124.835, 6.16484, 0.136532, 0);
  std::vector<double> x = {
      909.4848, 909.4908, 909.4992, 909.51,   909.5232, 909.5388, 909.5568,
      909.5772, 909.6,    909.6252, 909.6528, 909.6828, 909.7152, 909.75,
      909.7872, 909.8268, 909.8688, 909.9131, 909.96,   910.0092, 910.0608,
      910.1147, 910.1711, 910.2299, 910.2911, 910.3547, 910.4207, 910.4891,
      910.5599, 910.6331, 910.7087, 910.7867, 910.8671, 910.9499, 911.0351,
      911.1227, 911.2127, 911.3051, 911.3998, 911.4971, 911.5967, 911.6987,
      911.803,  911.9099, 912.019,  912.1306, 912.2446, 912.361,  912.4798,
      912.601,  912.7245, 912.8506, 912.9789, 913.1097, 913.2429, 913.3785,
      913.5165, 913.6569, 913.7997, 913.9449, 914.0925, 914.2424, 914.3948,
      914.5496, 914.7068, 914.8664, 915.0284, 915.1927, 915.3596, 915.5287,
      915.7003, 915.8743, 916.0507, 916.2295, 916.4106, 916.5942, 916.7802,
      916.9686, 917.1594, 917.3525, 917.5482, 917.7461, 917.9465, 918.1492,
      918.3544, 918.562,  918.772,  918.9843, 919.1991, 919.4163, 919.6359,
      919.8578, 920.0822, 920.309,  920.5381, 920.7697, 921.0036, 921.24,
      921.4787};
  std::vector<double> y = {
      1128.67,  1128.67,  1128.67,  1128.67,  1128.671, 1128.671, 1128.671,
      1128.671, 1128.671, 1128.672, 1128.672, 1128.672, 1128.673, 1128.673,
      1128.674, 1128.674, 1128.675, 1128.675, 1128.676, 1128.676, 1128.677,
      1128.677, 1128.678, 1128.679, 1128.679, 1128.68,  1128.681, 1128.682,
      1128.682, 1128.683, 1128.684, 1128.685, 1128.686, 1128.687, 1128.688,
      1128.689, 1128.69,  1128.691, 1128.692, 1128.694, 1128.695, 1128.696,
      1128.697, 1128.698, 1128.7,   1128.701, 1128.703, 1128.704, 1128.705,
      1128.707, 1128.708, 1128.71,  1128.711, 1128.713, 1128.715, 1128.716,
      1128.718, 1128.72,  1128.722, 1128.724, 1128.725, 1128.727, 1128.729,
      1128.731, 1128.733, 1128.735, 1128.738, 1128.74,  1128.742, 1128.744,
      1128.747, 1128.749, 1128.752, 1128.754, 1128.757, 1128.759, 1128.762,
      1128.765, 1128.768, 1128.771, 1128.774, 1128.777, 1128.78,  1128.783,
      1128.786, 1128.79,  1128.793, 1128.797, 1128.8,   1128.804, 1128.808,
      1128.812, 1128.816, 1128.82,  1128.824, 1128.829, 1128.833, 1128.838,
      1128.842};
  double end_path_s = 136.8334;
  double end_path_d = 6.096553;

  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  Path prev_path_map = PathFromVectors(x, y);
  ValidateDerivatives(prev_path_map, 1.0 / 50.0);
  Path ref_path = GenerateReferencePath(prev_path_map, state,
                                        std::make_tuple(end_path_s, end_path_d),
                                        state.Lane(), map_state);
}

TEST(TimeSampledPath, GenerateSDCPathByTimeSamples) {
  CarState state(909.481, 1128.67, 124.835, 6.16484, 0.136532, 0);
  std::vector<double> x = {
      909.485, 909.491, 909.499, 909.51,  909.523, 909.539, 909.557, 909.577,
      909.6,   909.625, 909.653, 909.683, 909.715, 909.75,  909.787, 909.827,
      909.869, 909.913, 909.96,  910.009, 910.061, 910.115, 910.171, 910.23,
      910.291, 910.355, 910.421, 910.489, 910.56,  910.633, 910.709, 910.787,
      910.867, 910.95,  911.035, 911.123, 911.213, 911.305, 911.4,   911.497,
      911.597, 911.699, 911.803, 911.91,  912.019, 912.131, 912.245, 912.361,
      912.48,  912.601, 912.725, 912.851, 912.979, 913.11,  913.243, 913.379,
      913.516, 913.657, 913.8,   913.945, 914.092, 914.242, 914.395, 914.55,
      914.707, 914.866, 915.028, 915.193, 915.36,  915.529, 915.7,   915.874,
      916.051, 916.23,  916.411, 916.594, 916.78,  916.969, 917.159, 917.352,
      917.548, 917.746, 917.947, 918.149, 918.354, 918.562, 918.772, 918.984,
      919.199, 919.416, 919.636, 919.858, 920.082, 920.309, 920.538, 920.77,
      921.004, 921.24,  921.479};
  std::vector<double> y = {
      1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67,
      1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67,
      1128.67, 1128.67, 1128.68, 1128.68, 1128.68, 1128.68, 1128.68, 1128.68,
      1128.68, 1128.68, 1128.68, 1128.68, 1128.68, 1128.68, 1128.68, 1128.68,
      1128.69, 1128.69, 1128.69, 1128.69, 1128.69, 1128.69, 1128.69, 1128.69,
      1128.69, 1128.7,  1128.7,  1128.7,  1128.7,  1128.7,  1128.7,  1128.7,
      1128.7,  1128.71, 1128.71, 1128.71, 1128.71, 1128.71, 1128.71, 1128.72,
      1128.72, 1128.72, 1128.72, 1128.72, 1128.72, 1128.73, 1128.73, 1128.73,
      1128.73, 1128.73, 1128.74, 1128.74, 1128.74, 1128.74, 1128.75, 1128.75,
      1128.75, 1128.75, 1128.76, 1128.76, 1128.76, 1128.77, 1128.77, 1128.77,
      1128.77, 1128.78, 1128.78, 1128.78, 1128.79, 1128.79, 1128.79, 1128.8,
      1128.8,  1128.8,  1128.81, 1128.81, 1128.82, 1128.82, 1128.82, 1128.83,
      1128.83, 1128.84, 1128.84};
  double end_path_s = 136.833;
  double end_path_d = 6.09655;
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  constexpr double time_step = 1.0 / 50.0;

  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map = PathFromVectors(x, y);

  Path ref_path = GenerateReferencePath(prev_path_map, state,
                                        std::make_tuple(end_path_s, end_path_d),
                                        state.Lane(), map_state);
  Path drivable_path = GenerateSDCPathByTimeSamples(
      map_state, ref_path, prev_path_map, sdc_state, 5.0, mph_to_mps(49.5),
      time_step, 1.0);

  ValidateDerivatives(drivable_path, time_step);
  const double dist = Distance(drivable_path.front(), drivable_path.back());
  GTEST_ASSERT_GE(dist, 0.0);
}

TEST(TimeSampledPath, GenerateSDCPathByTimeSamplesNoAccel) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(
      prev_path_map, sdc_state, std::make_tuple(sdc_state.s(), sdc_state.d()),
      sdc_state.Lane(), map_state);

  Path drivable_path = GenerateSDCPathByTimeSamples(
      map_state, ref_path, prev_path_map, sdc_state, 0.0, mph_to_mps(49.5),
      1.0 / 50.0, 1.0);
  std::cout << "front=" << drivable_path.front()
            << " back=" << drivable_path.back();
  const double dist = Distance(drivable_path.front(), drivable_path.back());
  EXPECT_LT(dist, 0.001);
}

TEST(TimeSampledPath, GenerateSDCPathByTimeSamplesBraking) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(
      prev_path_map, sdc_state, std::make_tuple(sdc_state.s(), sdc_state.d()),
      sdc_state.Lane(), map_state);

  Path drivable_path = GenerateSDCPathByTimeSamples(
      map_state, ref_path, prev_path_map, sdc_state, -2.0, mph_to_mps(49.5),
      1.0 / 50.0, 1.0);
  const double dist = Distance(drivable_path.front(), drivable_path.back());
  EXPECT_LT(dist, 0.001);
}

TEST(GeneratePaths, GenerateMultipleReferencePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(
      prev_path_map, sdc_state, std::make_tuple(sdc_state.s(), sdc_state.d()),
      sdc_state.Lane(), map_state);

  Path drivable_path = GenerateSDCPathByTimeSamples(
      map_state, ref_path, prev_path_map, sdc_state, 5.0, mph_to_mps(49.5),
      1.0 / 50.0, 1.0);

  Path ref_path2 = GenerateReferencePath(
      drivable_path, sdc_state, std::make_tuple(sdc_state.s(), sdc_state.d()),
      sdc_state.Lane(), map_state);

  drivable_path.erase(drivable_path.begin(), drivable_path.begin() + 3);
  Path drivable_path2 = GenerateSDCPathByTimeSamples(
      map_state, ref_path2, prev_path_map, sdc_state, 5.0, mph_to_mps(49.5),
      1.0 / 50.0, 1.0);
}

TEST(GeneratePaths, CalculateCostMoveOneMeterLessThanDontMove) {
  std::vector<Path> others;
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);

  Path sdc_path1;
  sdc_path1.emplace_back(0.0, 0.0, 0.0);
  sdc_path1.emplace_back(1.0, 0.0, 0.0);
  double cost_move = PathCost(sdc_state, sdc_path1, others, 0.2);

  Path sdc_path2;
  sdc_path2.emplace_back(0.0, 0.0, 0.0);
  sdc_path2.emplace_back(0.0, 0.0, 0.0);
  double cost_dont_move = PathCost(sdc_state, sdc_path2, others, 0.2);

  GTEST_ASSERT_GE(cost_dont_move, cost_move);
}

TEST(GeneratePaths, CalculateCostCloserToOther) {
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path other_path1;
  other_path1.emplace_back(5.0, 0.0, 0.0);
  other_path1.emplace_back(6.0, 0.0, 0.0);

  std::vector<Path> others;
  others.push_back(other_path1);

  Path sdc_path1;
  sdc_path1.emplace_back(1.0, 0.0, 0.0);
  sdc_path1.emplace_back(2.0, 0.0, 0.0);
  double cost_closer = PathCost(sdc_state, sdc_path1, others, 0.2);

  Path sdc_path2;
  sdc_path2.emplace_back(0.0, 0.0, 0.0);
  sdc_path2.emplace_back(1.0, 0.0, 0.0);
  double cost_further = PathCost(sdc_state, sdc_path2, others, 0.2);

  GTEST_ASSERT_GE(cost_closer, cost_further);
}

TEST(GeneratePaths, CalculateCostBothCollide) {
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path other_path1;
  other_path1.emplace_back(5.0, 0.0, 0.0);
  other_path1.emplace_back(5.0, 0.0, 0.0);
  other_path1.emplace_back(5.0, 0.0, 0.0);
  other_path1.emplace_back(5.0, 0.0, 0.0);
  other_path1.emplace_back(5.0, 0.0, 0.0);

  std::vector<Path> others;
  others.push_back(other_path1);

  Path sdc_path1;
  sdc_path1.emplace_back(4.0, 0.0, 0.0);
  sdc_path1.emplace_back(5.0, 0.0, 0.0);
  sdc_path1.emplace_back(6.0, 0.0, 0.0);
  sdc_path1.emplace_back(7.0, 0.0, 0.0);
  sdc_path1.emplace_back(8.0, 0.0, 0.0);
  double cost_earlier_collision = PathCost(sdc_state, sdc_path1, others, 0.2);

  Path sdc_path2;
  sdc_path2.emplace_back(2.0, 0.0, 0.0);
  sdc_path2.emplace_back(3.0, 0.0, 0.0);
  sdc_path2.emplace_back(4.0, 0.0, 0.0);
  sdc_path2.emplace_back(5.0, 0.0, 0.0);
  sdc_path2.emplace_back(6.0, 0.0, 0.0);
  double cost_later_collision = PathCost(sdc_state, sdc_path2, others, 0.2);

  GTEST_ASSERT_LT(cost_later_collision, cost_earlier_collision);
}
