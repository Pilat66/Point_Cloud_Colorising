// Кубовый режим раскраски карты (см. ColoriseMap/README.md).
//
// Карта (LAS с gps_time) делится на кубы 1 м; для каждого кадра определяется
// список кубов в его конусе внимания, затем точки каждого куба получают цвет от
// кадра с минимальным баллом
//
//   score = w_t * |t_photo - t_point| / T_ref + w_d * min(1, d / D_ref)
//
// (w_t, w_d, T_ref, D_ref настраиваются). Член расстояния ограничен, поэтому
// время доминирует при большой разнице во времени, а поиск можно прерывать,
// как только временной член превысил текущий лучший балл.
//
// Пик памяти ограничен: точки хранятся в исходных int32 (12 Б) + gps_time
// (8 Б) + intensity (2 Б); промежуточные массивы кадра не материализуются.
#pragma once

#include <string>

struct CubeOptions {
    std::string cloud_path;       // LAS с gps_time (обязателен)
    std::string photos_dir;       // каталог снимков
    std::string trajectory_path;  // позы лидара
    std::string calib_path;       // camera-блок + экстраинсик
    std::string output_path;      // LAS 1.4 fmt 7 (RGB + gps_time)

    std::string extrinsic_name;
    std::string extrinsic_direction;
    std::string euler_order = "xyz";
    std::string euler_units = "auto";
    double      time_shift     = 0.0;
    double      time_tolerance = 1.0;

    double cube_size            = 1.0;   // --cube
    bool   occlusion            = true;
    double occlusion_cell_px    = 1.0;   // --occlusion-cell
    double occlusion_depth_tol  = 0.03;  // --occlusion-depth-tol
    // --occlusion-max-depth / --occlusion-ray-margin: «нет лидарных лучей».
    double occlusion_max_depth  = 40.0;
    int    occlusion_ray_margin = 1;
    double min_camera_dist      = 0.5;   // --min-camera-dist
    double max_view_angle_deg   = 75.0;  // --max-view-angle
    double max_range            = 0.0;   // --map-max-range (0 = без отсечки)

    double score_w_time         = 1.0;   // --score-time-weight
    double score_w_dist         = 1.0;   // --score-dist-weight
    double score_t_ref          = 1.0;   // --score-time-scale, с
    double score_d_ref          = 10.0;  // --score-dist-scale, м

    bool   keep_uncolored       = false;
    int    jobs                 = 0;     // 0 = все ядра

    // --debug1 <csv>: CSV «какой кадр дал цвет какой точке» (точки — по gps_time;
    // строки только для окрашенных точек, в порядке кубов).
    std::string debug1_path;
};

// Возвращает 0 при успехе, бросает std::runtime_error при ошибке.
int runCubeColourise(const CubeOptions& o);
