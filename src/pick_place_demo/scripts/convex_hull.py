import trimesh
import coacd
import os
from pathlib import Path

def convex_decomposition(input_file, output_dir, threshold=0.01, merge=True):
    """凸分解"""
    mesh = trimesh.load(input_file, force="mesh")
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)

    # parts = coacd.run_coacd(mesh) # a list of convex hulls.
    parts = coacd.run_coacd(mesh, threshold=threshold, real_metric=True)

    p = Path(input_file)
    file_name = p.stem  # 文件名, 不含扩展名
    ext = p.suffix  # 扩展名

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    if merge:
        merged_mesh = merge_parts(parts)
        output_path = os.path.join(output_dir, f"{file_name}_dec{ext}")
        merged_mesh.export(output_path)
        print(f"Exported merged convex parts to {output_path}")
    else:
        for i, part in enumerate(parts):
            part_mesh = trimesh.Trimesh(vertices=part[0], faces=part[1])
            part_mesh.export(os.path.join(output_dir, f"{file_name}_{i}{ext}"))

        print(f"Exported {len(parts)} convex parts to {output_dir}")

def merge_parts(parts):
    """将分解后的网格合并成单个文件"""
    part_meshes = []
    for part in parts:
        # 将 coacd 的数据结构转回 trimesh
        m = trimesh.Trimesh(vertices=part[0], faces=part[1])
        part_meshes.append(m)

    # 使用 trimesh.util.concatenate 将多个网格合并为一个
    print("正在合并网格...")
    merged_mesh = trimesh.util.concatenate(part_meshes)

    return merged_mesh

if __name__ == "__main__":
    input_file = '../model/assets/track_link.stl'
    output_dir = '../model/assets'

    convex_decomposition(input_file, output_dir, merge=False)