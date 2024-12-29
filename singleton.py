def singleton(cls):
    instances = {}

    def get_instance(*args, **kwargs):
        if cls in instances:
            raise RuntimeError(f"An instance of {cls.__name__} already exists!")
        instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance