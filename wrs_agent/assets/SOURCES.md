# Asset provenance

FAFU URDF and seven STL link meshes were copied unchanged from:
https://github.com/FAFU-Robotics/fafu_arm_sdk
at local source commit `4f4a246efe97c51ded89d3fd8a9b9a595238e77f`,
directory `fafu_robot_python/fafu_robot_description`.
Only model description resources are included; no motor/serial control SDK is imported.
No standalone LICENSE file was found at the source repository root during packaging.
This project does not assign a new license to those third-party assets; retain attribution
and confirm the original author's redistribution terms when publishing the models.

Kitchen STL inputs were supplied by the user (pot, cooker, lid, cup, skimmer and plate).
Their original authors/licenses were not supplied. The import interprets their coordinates
as millimetres and converts them to metres. Skimmer orientation is normalized to lie on
the table. The plate block is a generated 100 x 60 x 30 mm box.

Runtime uses only repository-contained relative asset IDs. No original desktop paths,
credentials or absolute paths to the original SDK are required by the default scene.

STL assets are visual geometry, not validated contact/collision models. Blender material
appearance is richer than the WRS renderer; scene geometry and placement are shared.

## Kitchen expansion, 2026-10-17

User-supplied 大锅.stl and 大锅盖.stl replace the original pot/lid.
捞网架.stl, 碗的支架.stl and 碗架.stl add three separate supports.
The runtime filenames are large_pot.stl, large_pot_lid.stl, skimmer_holder.stl,
bowl_support.stl and bowl_rack.stl. The scene object IDs pot and pot_lid remain stable.

The five original millimetre-coordinate STL files are preserved under
`design/kitchen/source_stl/` using the same English basenames. Runtime files under
`wrs_agent/assets/kitchen/meshes/` are converted to metres by 0.001, retain their
horizontal CAD origin, and have their bottom at local z=0. These are separate
design originals and runtime resources, not two runtime copies. Source/runtime
SHA-256 hashes, dimensions and geometric placement checks are recorded in
`design/kitchen/import_manifest.json`. No source licence was supplied.

## Combined bowl assembly replacement, 2026-10-17

User-supplied 碗与支架整体.stl replaces the runtime bowl_support.stl mesh.
The original is preserved as design/kitchen/source_stl/bowl_with_support.stl.
The previous source_stl/bowl_support.stl remains a historical design backup only.
The new mesh is scaled by 0.001 and shifted +0.035 m locally in z so its bottom is zero.
The bowl assembly is centered on bowl_rack, with its support surface at the rack top
z=0.075 m; the bowl extends inside the ring. Geometry-based placement is static,
not a simulated physical constraint. Current hashes and bounds are in the manifest.

Four pale-yellow food blocks (32 x 18 x 22 mm) are generated box primitives,
configured directly in scene.toml as bowl_food_1 through bowl_food_4. They are
static visual examples in bowl_support_2 and require no external STL asset.
