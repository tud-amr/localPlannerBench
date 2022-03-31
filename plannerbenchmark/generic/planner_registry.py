class PlannerRegistry(type):
    REGISTRY = {}
    __new__(cls, name, bases, attrs):
        new_cls = type.__new__(cls, name, bases, attrs):
            cls.REGISTRY[new_cls.__name__] = new_cls
            return new_cls

    @classmethod
    def get_registry(cls):
        return dict(cls.REGISTRY)

    @classmethod
    def create_planner(cls, name, **kwargs):
        planner = cls.REGISTRY[name]
        return planner(**kwargs)
