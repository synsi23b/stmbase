from pathlib import Path
import os
import argparse
import sys


def main():
    chips = {
        1: 'stm32f103c8',
        2: 'stm32f401ccu6',
        3: 'stm8s103',
        4: 'stm32g030f6p6'
    }
    is_stm8 = [3]
    chiphelp = "\nAvailable chips: \n"
    for k, v in chips.items():
        chiphelp += f"{k}:\t{v}\n"

    parser = argparse.ArgumentParser(
        description='Generate a segger embedded studio or iar workbench project according to target processor')
    parser.add_argument(

        'chip', type=int, help="for which chip to initialize the project (by number)")
    parser.add_argument(
        '--heap', type=int, default=8192, help="how much bytes of heap to allocate. Default 8kb, ignored for stm8")

    parser.print_help()
    print(chiphelp)
    args = parser.parse_args()

    if args.chip not in chips:
        print("Invalid chip chosen, aborting")
        exit(-1)

    chipname = chips[args.chip]
    submodule_folder = Path(__file__).parent
    submodule_folder = submodule_folder.absolute()
    project_folder = submodule_folder.parent
    project_name = project_folder.name

    # copy static project base assets depending on the operating system
    if os.name == "nt":
        os.system(
            f"robocopy {submodule_folder}\\copy_into_base\\{chipname} {project_folder} /E")
    else:
        os.system(
            f"cp -rvf {submodule_folder}/copy_into_base/{chipname}/. {project_folder}")

    workspace_template_file = None
    if args.chip in is_stm8:
        workspace_template_file = submodule_folder / \
            f"synos8/iar_workspace_template.eww"
        workspace_file = project_folder / f"{project_name}_workspace.eww"
        project_template_file = submodule_folder / f"synos8/iar_project_template.ewp"
        project_file = project_folder / f"{project_name}.ewp"
    else:
        project_template_file = submodule_folder / \
            f"embos/{chipname}/project_template.emProject"
        project_file = project_folder / f"{project_name}.emProject"

    if workspace_template_file:
        with workspace_template_file.open('r') as f:
            template = f.read()
        with workspace_file.open('w') as f:
            f.write(template.format(project=project_name))

    with project_template_file.open('r') as f:
        template = f.read()
    with project_file.open('w') as f:
        f.write(template.format(project=project_name, heap_size=args.heap))


if __name__ == "__main__":
    main()
