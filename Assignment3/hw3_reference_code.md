# Assignment3 参考代码备份

编译运行方式：

```bash
cd Assignment3/Code/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
./Rasterizer output.png normal
./Rasterizer output.png phong
./Rasterizer output.png texture
./Rasterizer output.png bump
./Rasterizer output.png displacement
```

---

## 1. get_projection_matrix (main.cpp)

```cpp
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection;

    float angel = eye_fov / 180.0 * MY_PI;
    float t = zNear * std::tan(angel / 2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    Eigen::Matrix4f MorthoScale(4, 4);
    MorthoScale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zFar - zNear), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f MorthoPos(4, 4);
    MorthoPos << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f Mpersp2ortho(4, 4);
    Mpersp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // 牛的正向修正（乘 -1 翻转 z）
    Eigen::Matrix4f Mt(4, 4);
    Mt << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    Mpersp2ortho = Mpersp2ortho * Mt;
    projection = MorthoScale * MorthoPos * Mpersp2ortho * projection;

    return projection;
}
```

---

## 2. rasterize_triangle (rasterizer.cpp)

```cpp
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &view_pos)
{
    auto v = t.toVector4();

    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    min_x = std::floor(min_x);
    max_x = std::ceil(max_x);
    min_y = std::floor(min_y);
    max_y = std::ceil(max_y);

    for (int x = min_x; x < max_x; x++)
    {
        for (int y = min_y; y < max_y; y++)
        {
            if (insideTriangle(x, y, t.v))
            {
                float min_depth = FLT_MAX;
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                zp *= Z;
                min_depth = std::min(min_depth, zp);
                if (min_depth < depth_buf[get_index(x, y)])
                {
                    auto interpolated_color = alpha * t.color[0] + beta * t.color[1] + gamma * t.color[2];
                    auto interpolated_normal = alpha * t.normal[0] + beta * t.normal[1] + gamma * t.normal[2];
                    auto interpolated_texcoords = alpha * t.tex_coords[0] + beta * t.tex_coords[1] + gamma * t.tex_coords[2];
                    auto interpolated_shadingcoords = alpha * view_pos[0] + beta * view_pos[1] + gamma * view_pos[2];
                    fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;
                    auto pixel_color = fragment_shader(payload);
                    depth_buf[get_index(x, y)] = min_depth;
                    Eigen::Vector2i point(x, y); 
                    set_pixel(point, pixel_color);
                }
            }
        }
    }
}
```

---

## 3. phong_fragment_shader (main.cpp)

```cpp
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light : lights)
    {
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();

        // 环境光
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        // 距离衰减
        float r2 = (light.position - point).dot(light.position - point);
        Eigen::Vector3f I_r2 = light.intensity / r2;

        // 漫反射
        Eigen::Vector3f Ld = kd.cwiseProduct(I_r2);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

        // 高光（Blinn-Phong 用 half vector）
        Eigen::Vector3f Ls = ks.cwiseProduct(I_r2);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_dir)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}
```

---

## 4. texture_fragment_shader (main.cpp)

```cpp
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto &light : lights)
    {
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        float r2 = (light.position - point).dot(light.position - point);
        Eigen::Vector3f I_r2 = light.intensity / r2;

        Eigen::Vector3f Ld = kd.cwiseProduct(I_r2);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

        Eigen::Vector3f Ls = ks.cwiseProduct(I_r2);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_dir)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}
```

与 phong 的区别：`kd = payload.color` → `kd = texture_color / 255.f`，即漫反射颜色从纹理查。

---

## 5. bump_fragment_shader (main.cpp)

```cpp
Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    // 切线 t 和副法线 b
    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    // TBN 矩阵（切线空间 → 世界空间）
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    // 计算高度图梯度
    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    // 扰动法向量
    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    normal = TBN * ln;

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal.normalized();

    return result_color * 255.f;
}
```

注意：bump shader 不计算光照，而是直接把扰动后的法向量可视化（法向量 → 颜色）。

---

## 6. displacement_fragment_shader (main.cpp)

```cpp
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);

    // 位移：沿法向量移动顶点
    point += (kn * normal * payload.texture->getColor(u, v).norm());

    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto &light : lights)
    {
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        float r2 = (light.position - point).dot(light.position - point);
        Eigen::Vector3f I_r2 = light.intensity / r2;

        Eigen::Vector3f Ld = kd.cwiseProduct(I_r2);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

        Eigen::Vector3f Ls = ks.cwiseProduct(I_r2);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_dir)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}
```

与 bump 的区别：
1. bump 只扰动法向量，displacement 还要移动顶点位置
2. bump 不计算光照（直接输出法向量颜色），displacement 计算完整 Blinn-Phong 光照

---

## 辅助：light 结构体

```cpp
struct light
{
    Eigen::Vector3f position;   // 光源位置
    Eigen::Vector3f intensity;  // 光源强度
};
```

## 辅助：fragment_shader_payload (Shader.hpp)

```cpp
struct fragment_shader_payload
{
    Eigen::Vector3f view_pos;    // 像素在相机空间的坐标
    Eigen::Vector3f color;       // 插值后的颜色
    Eigen::Vector3f normal;      // 插值后的法向量
    Eigen::Vector2f tex_coords;  // 插值后的uv纹理坐标
    Texture* texture;            // 纹理图指针
};
```
