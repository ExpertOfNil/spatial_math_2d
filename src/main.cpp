#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include "transform.hpp"

typedef std::array<float, 9> Matrix3;

void DrawGrid2d(Vector2 screen, float spacing, Color color) {
    float x_pos = 0;
    while (x_pos < screen.x) {
        DrawLineV({x_pos, 0.0f}, {x_pos, screen.y}, color);
        x_pos += spacing;
    }
    float y_pos = 0;
    while (y_pos < screen.y) {
        DrawLineV({0.0f, y_pos}, {screen.x, y_pos}, color);
        y_pos += spacing;
    }
}

void DrawAxis(Transform2d xform, float scale, float thk) {
    float fthk = thk * 2.0f;
    float arr_tip_offset = scale * 0.2f + fthk;
    if (scale < 10.0f) {
        arr_tip_offset = 2.0f;
        scale = 10.0f;
    }
    arr_tip_offset =
        arr_tip_offset > 20.0 + fthk ? 20.0 + fthk : arr_tip_offset;
    float arr_side_offset = arr_tip_offset * 0.3f;
    float z = xform.z_mag();
    if (z > 0) {
        arr_side_offset = -arr_side_offset;
    }

    cv::Point2f origin = xform.local_to_world(cv::Point2f(0.0f, 0.0f));
    Vector2 origin_v = {origin.x, origin.y};

    cv::Point2f x_axis_end = xform.local_to_world(cv::Point2f(scale, 0.0f));
    DrawLineEx(origin_v, {x_axis_end.x, x_axis_end.y}, thk, RED);
    cv::Point2f x_arrow_tip[3] = {
        xform.local_to_world(cv::Point2f(scale + arr_tip_offset, 0.0f)),
        xform.local_to_world(cv::Point2f(scale, arr_side_offset)),
        xform.local_to_world(cv::Point2f(scale, -arr_side_offset)),
    };
    DrawTriangle(
        {x_arrow_tip[0].x, x_arrow_tip[0].y},
        {x_arrow_tip[1].x, x_arrow_tip[1].y},
        {x_arrow_tip[2].x, x_arrow_tip[2].y},
        RED);

    cv::Point2f y_axis_end = xform.local_to_world(cv::Point2f(0.0f, scale));
    DrawLineEx(origin_v, {y_axis_end.x, y_axis_end.y}, thk, GREEN);
    cv::Point2f y_arrow_tip[3] = {
        xform.local_to_world(cv::Point2f(0.0f, scale + arr_tip_offset)),
        xform.local_to_world(cv::Point2f(-arr_side_offset, scale)),
        xform.local_to_world(cv::Point2f(arr_side_offset, scale)),
    };
    cv::Point2f y_test[3] = {
        cv::Point2f(0.0f, scale + arr_tip_offset),
        cv::Point2f(-arr_side_offset, scale),
        cv::Point2f(arr_side_offset, scale),
    };
    DrawTriangle(
        {y_arrow_tip[0].x, y_arrow_tip[0].y},
        {y_arrow_tip[1].x, y_arrow_tip[1].y},
        {y_arrow_tip[2].x, y_arrow_tip[2].y},
        GREEN);

    DrawCircleV(origin_v, 5.0f, LIGHTGRAY);
}

void DrawKeypoints(std::vector<Vector2>& keypoints, float radius) {
    bool first_done = false;
    for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
        if (!first_done) {
            first_done = true;
            continue;
        }
        DrawCircleV({(*it).x, (*it).y}, radius, PURPLE);
    }
    DrawCircleV({keypoints[0].x, keypoints[0].y}, radius, YELLOW);
}

int main(int argc, char* argv[]) {
    int screen_w = 960;
    int screen_h = 540;

    cv::Vec4f fit_line = {0.020584, 0.999788, 1233.198242, 1766.562988};
    cv::Point2f ht_ref_pt = {1216.782104, 969.212341};
    const Transform2d tform(fit_line, ht_ref_pt);
    std::vector<Vector2> keypoints = {
        {ht_ref_pt.x, ht_ref_pt.y},
        {.x = 1217.944702, .y = 969.188354},
        {.x = 1217.968872, .y = 1038.283081},
        {.x = 1220.063477, .y = 1110.561890},
        {.x = 1221.548340, .y = 1179.118530},
        {.x = 1221.904907, .y = 1247.586548},
        {.x = 1224.523560, .y = 1319.157227},
        {.x = 1225.589844, .y = 1386.809692},
        {.x = 1225.723633, .y = 1454.988647},
        {.x = 1228.762207, .y = 1525.779541},
        {.x = 1229.764893, .y = 1592.691284},
        {.x = 1229.559692, .y = 1637.239380},
        {.x = 1231.335327, .y = 1707.557983},
        {.x = 1234.042847, .y = 1775.953735},
        {.x = 1233.513062, .y = 1841.752197},
        {.x = 1236.902100, .y = 1911.643311},
        {.x = 1237.816406, .y = 1977.321167},
        {.x = 1236.614990, .y = 2020.174194},
        {.x = 1239.232544, .y = 2088.850830},
        {.x = 1241.805298, .y = 2154.531494},
        {.x = 1240.922241, .y = 2217.188721},
        {.x = 1244.093628, .y = 2284.854004},
        {.x = 1246.845093, .y = 2350.178711},
        {.x = 1246.509521, .y = 2391.842041},
        {.x = 1246.901855, .y = 2457.331299},
        {.x = 1250.066284, .y = 2523.489746},
    };

    float min_x = 5000.0f;
    float min_y = 5000.0f;
    float max_x = 0.0f;
    float max_y = 0.0f;
    for (auto& kp : keypoints) {
        if (kp.x < min_x) min_x = kp.x;
        if (kp.x > max_x) max_x = kp.x;
        if (kp.y < min_y) min_y = kp.y;
        if (kp.y > max_y) max_y = kp.y;
    }

    cv::Point2f pt(100.0f, 100.0f);
    cv::Point2f pt_l = tform.world_to_local(pt);

    screen_w = 2160;
    screen_h = 3840;
    const Vector2 screen = {(float)screen_w, (float)screen_h};
    Camera2D camera = {
        .offset = {0.0f, 0.0f},
        .target = {0.0f, 0.0f},
        .rotation = 0.0f,
        .zoom = 0.4f,
    };

    Transform2d world;
    InitWindow(screen_w * camera.zoom, screen_h * camera.zoom, "Visualizer 2D");
    SetTargetFPS(60);
    bool move_pt;
    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MouseButton::MOUSE_BUTTON_LEFT)) {
            Vector2 mouse_pos = GetMousePosition();
            pt.x = mouse_pos.x / camera.zoom;
            pt.y = mouse_pos.y / camera.zoom;
            pt_l = tform.world_to_local(pt);
        }
        BeginDrawing();
        {
            ClearBackground(Color{.r = 18, .g = 18, .b = 18, .a = 255});

            BeginMode2D(camera);
            {
                float pt_radius = 5.0f / camera.zoom;
                DrawGrid2d(screen, 10.0f, DARKGRAY);
                DrawAxis(world, 500.0f, 4.0f / camera.zoom);
                DrawAxis(tform, 200.0f, 2.0f / camera.zoom);
                DrawCircleV({pt.x, pt.y}, pt_radius, PINK);
                DrawKeypoints(keypoints, pt_radius);
                DrawLineV({min_x, min_y}, {max_x, max_y}, LIGHTGRAY);
                const char* fit_lbl =
                    TextFormat("FIT: (%.4f, %.4f)", pt_l.x, pt_l.y);
                DrawText(
                    fit_lbl,
                    pt.x + 20.0f,
                    pt.y + 20.0f,
                    20 / camera.zoom,
                    SKYBLUE);
            }
            EndMode2D();
            const char* world_lbl =
                TextFormat("WORLD: (%.4f, %.4f)", pt.x, pt.y);
            DrawText(world_lbl, 20, 20, 20, SKYBLUE);
        }
        EndDrawing();
    }
    return 0;
}
