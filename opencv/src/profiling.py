import time, atexit, numpy
from typing import Union, Tuple, Callable

timing_histogram = {}
registered_names = set()

def time_this(func, alias: Union[None, str] = None) -> Callable:
    """
    Decorator that tracks the duration of every call to the wrapped function. Takes an optional function alias.

    Throws if multiple functions with the same name are registered.
    Note that the details of every call are stored perpetually. If a long runtime is expected, empty **timing_histogram** regularly.
    
    See **calc_performance_info** and **__print_result** for getting performance info.
    """

    func_name = alias if alias is not None else func.__name__

    if func_name in registered_names:
        raise TypeError(f"A function with the name '{func_name}' was registered with time_this twice")
    
    registered_names.add(func_name)
    timing_histogram[func_name] = []

    def timing_wrapper(*args, **kargs):

        t0 = time.time()
        result = func(*args, **kargs)
        timing_histogram[func_name].append(time.time()-t0)

        return result
    
    return timing_wrapper


def calc_performance_info(func_name: str) -> Union[None, Tuple[float, float, int]]:
    """
    Returns the current usage statistics of a function with name **func_name** or **None** if the function is not tracked.

    The usage statistics are in the following format; (avg. duration, 95% duration, total calls).
    """

    value = timing_histogram.get(func_name, None)
    
    if value is None:
        return None

    return (
        round(numpy.mean(value), 5),
        round(numpy.percentile(value, 95), 5),
        len(value)
    )


def __print_result() -> None:
    """
    Prints out the following details of all registered functions (sorted by average duration) at program exit;
        - function name/alias
        - average duration
        - 95% percentile duration
        - total calls
    Functions which were registered but never called are listed at the end.
    """ 

    results = []
    never_called = []

    for key, value in timing_histogram.items():
        if len(value) > 0:
            results.append(
                (
                    key,
                    round(numpy.mean(value), 5),
                    round(numpy.percentile(value, 95), 5),
                    len(value),
                )
            )
        else: never_called.append(key)

    ordered = sorted(results, key=lambda item: item[1], reverse=True)

    print("-====Timing Results====-")
    for i, value in enumerate(ordered):
        print(f"{i+1}) '{value[0]}' avg={value[1]} 95%={value[2]} calls={value[3]}")
    print("NOTE: The following functions were never called: " + str(never_called)[1:-1])


atexit.register(__print_result)
