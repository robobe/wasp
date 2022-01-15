import asyncio
import time
import threading

def start_cpu(load_time):
    print("start cpu load")
    print(threading.current_thread().name)
    time.sleep(load_time)
    print("end cpu load")

async def sleep_test():
    loop.run_in_executor(None, start_cpu, 5)

async def parallel():
    # run two sleep_tests in parallel and wait until both finish
    await asyncio.gather(sleep_test(), sleep_test())

loop = asyncio.get_event_loop()
loop.create_task(parallel())
loop.run_forever()