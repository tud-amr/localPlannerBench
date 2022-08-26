import re
import yaml


class PlannerRegistry(type):
    REGISTRY = {}
    def __new__(cls, name, bases, attrs):
        new_cls = type.__new__(cls, name, bases, attrs)
        name_match = re.match(r'(.*)Planner', name)
        aliases = [name, name_match.group(1), name_match.group(1).lower()]
        # TODO: Ugly workaround to respect dynamic and static fabrics <02-04-22, mspahn> #
        if 'fabric' in aliases:
            aliases.append('dynamicFabric')
        for alias in aliases:
            cls.REGISTRY[alias] = new_cls

        return new_cls

    @classmethod
    def get_registry(cls):
        return dict(cls.REGISTRY)

    @classmethod
    def parseSetup(cls, setupFile):
        with open(setupFile, "r") as setupStream:
            cls._setup = yaml.safe_load(setupStream)

    @classmethod
    def create_planner(cls, exp, setup, **kwargs):
        cls.parseSetup(setup)
        name = cls._setup['name']
        planner = cls.REGISTRY[name]
        return planner(exp, **cls._setup)
