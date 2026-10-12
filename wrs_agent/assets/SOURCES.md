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
