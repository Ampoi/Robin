import aiohttp_cors
from aiohttp import web

def runServer(on_shutdown, offer):
  app = web.Application()
  app.on_shutdown.append(on_shutdown)
  app.router.add_post("/offer", offer)

  cors = aiohttp_cors.setup(
      app,
      defaults={
          "*": aiohttp_cors.ResourceOptions(
              allow_credentials=True,
              expose_headers="*",
              allow_headers="*",
          )
      },
  )

  for route in list(app.router.routes()):
      cors.add(route)

  web.run_app(app, port=8080)