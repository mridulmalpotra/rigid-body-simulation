import os
import bpy

# put the location to the folder where the objs are located here in this fashion
path_to_obj_dir = os.path.join("/home/mty/cuda-workspace/RigidBodySimulation/src/obj/")

# get list of all files in directory
file_list = os.listdir(path_to_obj_dir)

# get a list of files ending in 'obj'
obj_list = [item for item in file_list if item[-3:] == 'obj']

# loop through the strings in obj_list 
for item in obj_list:
    # select all object
    bpy.ops.object.select_all(action='SELECT')
    # delete selected
    bpy.ops.object.delete(use_global=True)
    # import obj
    path_to_import = os.path.join(path_to_obj_dir, item)
    bpy.ops.import_scene.obj(filepath = path_to_import)
    # rename
    for obj in bpy.context.selected_objects:
        obj.name = item[:-4]
        print(dir(obj))


    # write blend
    path_to_export = os.path.join(path_to_obj_dir, item[:-4] + ".blend")
    print(path_to_export)
    bpy.ops.wm.save_as_mainfile(filepath=path_to_export, check_existing=False )