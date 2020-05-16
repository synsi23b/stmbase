from pathlib import Path
import os
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description='Generate a Rowley crossworks project according to target processor')
    parser.add_argument('chip', type=str, help="for which chip to initialize the project")
    parser.add_argument('heap', type=int, help="how much bytes of heap to allocate")
    args = parser.parse_args()
    #parser.print_help()

    submodule_folder = Path(__file__).parent
    project_folder = submodule_folder.parent
    
    os.system(f"cp -rf {submodule_folder}/copy_into_base/. {project_folder}")

    project_name = project_folder.name
    project_file = project_folder / f"{project_name}.hzp"
    template_file = submodule_folder / f"embos/{args.chip}/project_template.hzp"
    with template_file.open('r') as file:
        template = file.read()
    with project_file.open('w') as file:
        file.write(template.format(project=project_name, heap_size=args.heap))

if __name__ == "__main__":
    main()