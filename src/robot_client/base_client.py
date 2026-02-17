"""
BaseClient - Shared connection logic for all subsystem clients
"""

from typing import Dict


class BaseClient:
    """Base class for all subsystem clients - handles protocol connection."""

    def __init__(self, protocol='zmq', host='localhost', **kwargs):
        """
        Initialize client connection.

        Args:
            protocol: 'zmq', 'http', or 'websocket'
            host: Hostname or IP of container (default: 'localhost')
            **kwargs: Protocol-specific options
        """
        self.protocol = protocol
        self.host = host

        if protocol == 'zmq':
            from .protocols.zmq import ZMQClient
            command_port = kwargs.get('command_port', 5555)
            state_port = kwargs.get('state_port', 5556)
            self._client = ZMQClient(host, command_port, state_port)

        elif protocol == 'http':
            from .protocols.http import HTTPClient
            port = kwargs.get('port', 5000)
            self._client = HTTPClient(host, port)

        elif protocol == 'websocket' or protocol == 'ws':
            from .protocols.websocket import WebSocketClient
            port = kwargs.get('port', 5000)
            self._client = WebSocketClient(host, port)

        else:
            raise ValueError(f"Unknown protocol: {protocol}. Use 'zmq', 'http', or 'websocket'")

    def ping(self) -> float:
        """Test connection latency (ms)."""
        return self._client.ping()

    def close(self):
        """Close connection."""
        if hasattr(self._client, 'close'):
            self._client.close()

    def __enter__(self):
        """Support context manager."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanup when exiting context."""
        self.close()
