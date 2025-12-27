#!/bin/sh
set -e

# Skip default entrypoint script behavior
# Just start nginx with pid file in /tmp (writable by non-root user)
exec nginx -g "daemon off; pid /tmp/nginx.pid;"
