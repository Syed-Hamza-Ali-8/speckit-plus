#!/bin/sh
# Custom entrypoint that starts nginx directly
# This bypasses the default nginx:alpine entrypoint

exec /usr/sbin/nginx -g "daemon off;"
