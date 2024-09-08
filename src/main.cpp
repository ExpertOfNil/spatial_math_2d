#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <cstdio>
#include <opencv2/core.hpp>

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

void DrawAxis(Transform2d xform, float scale) {
    float arr_tip_offset = scale * 0.2f;
    if (scale < 10.0f) {
        arr_tip_offset = 2.0f;
        scale = 10.0f;
    }
    arr_tip_offset = arr_tip_offset > 20.0 ? 20.0 : arr_tip_offset;
    float arr_side_offset = arr_tip_offset * 0.3f;

    cv::Point2f origin = xform.local_to_world(cv::Point2f(0.0f, 0.0f));
    Vector2 origin_v = {origin.x, origin.y};

    cv::Point2f x_axis_end = xform.local_to_world(cv::Point2f(scale, 0.0f));
    DrawLineEx(origin_v, {x_axis_end.x, x_axis_end.y}, 2.0f, RED);
    cv::Point2f x_arrow_tip[3] = {
        xform.local_to_world(cv::Point2f(scale + arr_tip_offset, 0.0f)),
        xform.local_to_world(cv::Point2f(scale, -arr_side_offset)),
        xform.local_to_world(cv::Point2f(scale, arr_side_offset)),
    };
    DrawTriangle(
        {x_arrow_tip[0].x, x_arrow_tip[0].y},
        {x_arrow_tip[1].x, x_arrow_tip[1].y},
        {x_arrow_tip[2].x, x_arrow_tip[2].y},
        RED);

    cv::Point2f y_axis_end = xform.local_to_world(cv::Point2f(0.0f, scale));
    DrawLineEx(origin_v, {y_axis_end.x, y_axis_end.y}, 2.0f, GREEN);
    cv::Point2f y_arrow_tip[3] = {
        xform.local_to_world(cv::Point2f(0.0f, scale + arr_tip_offset)),
        xform.local_to_world(cv::Point2f(arr_side_offset, scale)),
        xform.local_to_world(cv::Point2f(-arr_side_offset, scale)),
    };
    DrawTriangle(
        {y_arrow_tip[0].x, y_arrow_tip[0].y},
        {y_arrow_tip[1].x, y_arrow_tip[1].y},
        {y_arrow_tip[2].x, y_arrow_tip[2].y},
        GREEN);

    DrawCircleV(origin_v, 5.0f, LIGHTGRAY);
}

int main(int argc, char* argv[]) {
    const int screen_w = 960;
    const int screen_h = 540;
    const Vector2 screen = {(float)screen_w, (float)screen_h};

    Camera2D camera = {
        .offset = {screen_w / 2.0f, screen_h / 2.0f},
        .target = {0.0f, 0.0f},
        .rotation = 0.0f,
        .zoom = 1.0f,
    };

    float theta = sin(PI / 6.0f);
    Vector2 yprime = {theta, 1.0f};
    yprime = Vector2Normalize(yprime);
    Vector2 xprime = {yprime.y, -yprime.x};
    xprime = Vector2Normalize(xprime);
    Vector2 oprime = {400.0f, 200.0f};
    Matrix3 xform = {
        // clang-format off
        xprime.x, xprime.y, 0.0f,
        yprime.x, yprime.y, 0.0f,
        oprime.x, oprime.y, 1.0f,
        // clang-format on
    };
    const Transform2d tform(xform);
    cv::Point2f oprime_l =
        tform.world_to_local(cv::Point2f(oprime.x, oprime.y));

    cv::Point2f pt(100.0f, 100.0f);
    cv::Point2f pt_l = tform.world_to_local(pt);

    InitWindow(screen_w, screen_h, "Visualizer 2D");
    SetTargetFPS(60);
    bool move_pt;
    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MouseButton::MOUSE_BUTTON_LEFT)) {
            Vector2 mouse_pos = GetMousePosition();
            pt.x = mouse_pos.x;
            pt.y = mouse_pos.y;
            pt_l = tform.world_to_local(pt);
        }
        BeginDrawing();
        {
            ClearBackground(Color{.r = 18, .g = 18, .b = 18, .a = 255});
            DrawGrid2d(screen, 10.0f, DARKGRAY);
            DrawAxis(tform, 200.0f);
            DrawCircleV({pt.x, pt.y}, 5.0f, PURPLE);

            // Stuff Drawn wrt the camera
            // BeginMode2D(camera);
            //{
            //}
            // EndMode2D();
            const char* world_msg =
                TextFormat("World: [%.4f, %.4f]", pt.x, pt.y);
            DrawText(world_msg, 20, 20, 20, SKYBLUE);
            const char* local_msg =
                TextFormat("Local: [%.4f, %.4f]", pt_l.x, pt_l.y);
            DrawText(local_msg, 20, 40, 20, SKYBLUE);
        }
        EndDrawing();
    }
    return 0;
}
