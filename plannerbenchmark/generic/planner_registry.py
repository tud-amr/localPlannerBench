import re
from hydra.core.config_store import ConfigStore
from dataclasses import dataclass


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
        cls._setup = setupFile

    @classmethod
    def create_planner(cls, exp, setup, **kwargs):
        cls.parseSetup(setup)
        name = cls._setup['name']
        planner = cls.REGISTRY[name]
        return planner(exp, **cls._setup)


cs = ConfigStore.instance()

class PlannerConfigRegistry(type):
    def __new__(cls, name, bases, attrs):
        new_cls = type.__new__(cls, name, bases, attrs)

        if "PlannerConfig" in [b.__name__ for b in bases]:
            # NOTE: Dirty trick to register proper dataclass to ConfigStore.
            dataclass(new_cls)
            name_match = re.match(r'(.*)Config', name)
            cs.store(group="planner", name=f"base_{name_match.group(1).lower()}", node=new_cls)
            new_cls = type.__new__(cls, name, bases, attrs)

        return new_cls

