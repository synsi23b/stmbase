from pathlib import Path
import os
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description='Generate a segger embedded studio or iar workbench project according to target processor')
    parser.add_argument('bits', type=str, help="STM8 or STM32")
    parser.add_argument('chip', type=str, help="for which chip to initialize the project")
    parser.add_argument('heap', type=int, help="how much bytes of heap to allocate")
    args = parser.parse_args()
    #parser.print_help()

    submodule_folder = Path(__file__).parent
    project_folder = submodule_folder.parent
    project_name = project_folder.name
    
    os.system(f"cp -rf {submodule_folder}/copy_into_base/{args.chip}/. {project_folder}")
    if args.bits == "STM8":
        pass
    elif args.bits == "STM32":
        template_file = submodule_folder / f"embos/{args.chip}/project_template.emProject"
        project_file = project_folder / f"{project_name}.emProject"
    else:
        raise ValueError("Unknown processor bits count")
    
    with template_file.open('r') as file:
        template = file.read()
    with project_file.open('w') as file:
        file.write(template.format(project=project_name, heap_size=args.heap))

if __name__ == "__main__":
    main()