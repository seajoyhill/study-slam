// Pangolin 烟测：交互 3D 视图 + RGB 轴线（线框箭头）+ 轴上文字（P*MV 投到 View 像素坐标后 Draw）。
#include <cmath>
#include <iostream>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/glfont.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/handler/handler.h>

namespace {

void GlDrawAxisWithArrows(float s) {
  const float wing = 0.12f * s;
  const float rad = 0.05f * s;
  const float t = s - wing;
  float cols[18];
  auto lines = [&](const float* v, float r, float g, float b) {
    for (int i = 0; i < 6; ++i) {
      cols[3 * i + 0] = r;
      cols[3 * i + 1] = g;
      cols[3 * i + 2] = b;
    }
    pangolin::glDrawColoredVertices<float, float>(6, v, cols, GL_LINES, 3, 3);
  };
  const float vx[] = {0, 0, 0, s, 0, 0, s, 0, 0, t, rad, 0, s, 0, 0, t, -rad, 0};
  lines(vx, 1, 0, 0);
  const float vy[] = {0, 0, 0, 0, s, 0, 0, s, 0, rad, t, 0, 0, s, 0, -rad, t, 0};
  lines(vy, 0, 1, 0);
  const float vz[] = {0, 0, 0, 0, 0, s, 0, 0, s, rad, 0, t, 0, 0, s, -rad, 0, t};
  lines(vz, 0, 0, 1);
}

// 世界点 -> 当前 glViewport 内像素（相对 View 左下），用 cam 的 P*MV（勿用 GlText::DrawWindow）
bool WorldToViewLocalPx(const pangolin::OpenGlRenderState& cam, float wx, float wy, float wz,
                        float* lx, float* ly) {
  const pangolin::OpenGlMatrix pm = cam.GetProjectionModelViewMatrix();
  const double* m = pm.m;
  const double x = wx, y = wy, z = wz;
  const double cx = m[0] * x + m[4] * y + m[8] * z + m[12];
  const double cy = m[1] * x + m[5] * y + m[9] * z + m[13];
  const double cw = m[3] * x + m[7] * y + m[11] * z + m[15];
  if (std::abs(cw) < 1e-12) {
    return false;
  }
  const double nx = cx / cw, ny = cy / cw;
  if (std::abs(nx) > 1.2 || std::abs(ny) > 1.2) {
    return false;
  }
  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  *lx = static_cast<float>((nx * 0.5 + 0.5) * vp[2]);
  *ly = static_cast<float>((ny * 0.5 + 0.5) * vp[3]);
  return true;
}

void DrawAxisLabels(pangolin::View& view, const pangolin::OpenGlRenderState& cam,
                    const pangolin::GlText& tx, const pangolin::GlText& ty, const pangolin::GlText& tz,
                    float u, float off) {
  float lx[3], ly[3];
  const bool ok[3] = {WorldToViewLocalPx(cam, u, off, 0.f, lx + 0, ly + 0),
                      WorldToViewLocalPx(cam, off, u, 0.f, lx + 1, ly + 1),
                      WorldToViewLocalPx(cam, 0.f, off, u, lx + 2, ly + 2)};
  const pangolin::GlText* lbl[3] = {&tx, &ty, &tz};
  const float rgb[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  view.ActivatePixelOrthographic();
  glMatrixMode(GL_MODELVIEW);
  for (int i = 0; i < 3; ++i) {
    if (!ok[i]) {
      continue;
    }
    glColor3f(rgb[i][0], rgb[i][1], rgb[i][2]);
    glLoadIdentity();
    glTranslatef(std::floor(lx[i]), std::floor(ly[i]), 0.f);
    lbl[i]->Draw();
  }
}

}  // namespace

int main() {
  constexpr int w = 640;
  constexpr int h = 480;

  pangolin::CreateWindowAndBind("pangolin_smoke", w, h);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(w, h, 420, 420, w / 2.0, h / 2.0, 0.2, 100),
      pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -static_cast<float>(w) / h)
                              .SetHandler(&handler);

  pangolin::GlFont font(28.f);
  const pangolin::GlText label_x = font.Text("x");
  const pangolin::GlText label_y = font.Text("y");
  const pangolin::GlText label_z = font.Text("z");

  std::cout << "pangolin smoke: RGB 轴 + 标注，拖拽旋转、滚轮缩放，关窗或 ESC 退出\n";

  while (!pangolin::ShouldQuit()) {
    glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    constexpr float axis_len = 1.5f;
    GlDrawAxisWithArrows(axis_len);

    const float u = 0.82f * axis_len;
    const float off = 0.12f * axis_len;

    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_SCISSOR_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    DrawAxisLabels(d_cam, s_cam, label_x, label_y, label_z, u, off);
    glPopAttrib();

    pangolin::FinishFrame();
  }

  std::cout << "pangolin smoke test: window loop exited ok\n";
  return 0;
}
