import asyncio
import time 

# Global storage variable
storage = []

# Some function - Controller or getting data
async def print_attitude(var):
    out = 1*var
    return out

# Main loop 
async def run():
    # Initialize values
    global storage
    ii = 0 
    timer = time.time()

    # Run forever till broken 
    while(True):
        # Small pause - 10 Hz
        await asyncio.sleep(0.1)
        
        # Perform the calculation
        A = await asyncio.ensure_future(print_attitude(ii))
        storage.append(A)
        
        # Print data
        print(1/(time.time()-timer), A)
        
        # Reset and icrement timer
        timer = time.time()
        ii += 1 
  
        
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
        
    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Caught keyboard interrupt.")    
        print(storage)    
    finally:
        loop.close()
        
        

