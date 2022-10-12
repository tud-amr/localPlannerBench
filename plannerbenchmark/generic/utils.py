import os
import logging

# Import custom planners
def import_custom_planners():
    paths = os.environ.get('LOCAL_PLANNER_BENCH_CUSTOM')
    if paths is None:
        logging.warn("Environment variable for custom planner directories not set")
        return

    paths = paths.split(":")

    import importlib.util
    import sys
    for i, path in enumerate(paths):
        spec = importlib.util.spec_from_file_location(f"customPlanners{i}", f"{path}/__init__.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module 
        spec.loader.exec_module(module)
