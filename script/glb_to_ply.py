import open3d as o3d
import trimesh
import numpy as np
import argparse
import os
import sys

def load_mesh_from_glb(glb_path):
    try:
        scene_or_mesh = trimesh.load(glb_path)
    except Exception as e:
        print(f"❌ 加载 GLB 文件失败: {e}")
        sys.exit(1)

    if isinstance(scene_or_mesh, trimesh.Scene):
        geometries = list(scene_or_mesh.geometry.values())
        if not geometries:
            print("❌ Scene 中未包含几何体")
            sys.exit(1)
        mesh_trimesh = trimesh.util.concatenate(geometries)
    else:
        mesh_trimesh = scene_or_mesh

    if mesh_trimesh.vertices is None or len(mesh_trimesh.vertices) == 0:
        print("❌ 模型中未包含有效顶点，无法转换。")
        sys.exit(1)

    return mesh_trimesh

def convert_to_pointcloud(mesh_trimesh, sample_points=100000):
    if isinstance(mesh_trimesh, trimesh.Trimesh):
        mesh_o3d = o3d.geometry.TriangleMesh()
        mesh_o3d.vertices = o3d.utility.Vector3dVector(mesh_trimesh.vertices)
        mesh_o3d.triangles = o3d.utility.Vector3iVector(mesh_trimesh.faces)
        mesh_o3d.compute_vertex_normals()

        # 采样成点云
        pcd = mesh_o3d.sample_points_uniformly(number_of_points=sample_points)
    elif isinstance(mesh_trimesh, trimesh.PointCloud):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(mesh_trimesh.vertices)
    else:
        print(f"❌ 不支持的对象类型: {type(mesh_trimesh)}")
        sys.exit(1)

    # 尝试添加颜色
    if hasattr(mesh_trimesh, 'colors') and mesh_trimesh.colors is not None:
        colors = np.array(mesh_trimesh.colors)
        if colors.shape[0] == len(mesh_trimesh.vertices):
            if colors.dtype != np.float32 and colors.dtype != np.float64:
                colors = colors.astype(np.float32) / 255.0
            if colors.shape[1] == 4:  # RGBA -> RGB
                colors = colors[:, :3]
            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            print("⚠️ 警告：颜色数量与顶点数量不一致，跳过颜色赋值")

    return pcd

def main():
    parser = argparse.ArgumentParser(description="Convert GLB model to point cloud")
    parser.add_argument("input_glb", help="Path to input .glb file")
    parser.add_argument("output_file", help="Output file path (.ply or .pcd)")
    parser.add_argument("--points", type=int, default=100000, help="Number of sampled points (for mesh only)")
    args = parser.parse_args()

    if not os.path.isfile(args.input_glb):
        print(f"❌ 输入文件不存在: {args.input_glb}")
        sys.exit(1)

    output_ext = os.path.splitext(args.output_file)[-1].lower()
    if output_ext not in [".ply", ".pcd"]:
        print("❌ 输出文件必须为 .ply 或 .pcd 格式")
        sys.exit(1)

    mesh_trimesh = load_mesh_from_glb(args.input_glb)
    pcd = convert_to_pointcloud(mesh_trimesh, sample_points=args.points)

    success = o3d.io.write_point_cloud(args.output_file, pcd)
    if success:
        print(f"✅ 转换成功: {args.output_file}")
    else:
        print("❌ 保存失败")

if __name__ == "__main__":
    main()
