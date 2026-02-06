#!/usr/bin/env python3.8
"""
Container Path Resolution for TossingBot
Detects container environment and resolves paths appropriately
"""
import os
from typing import Optional, Dict


class ContainerPathResolver:
    """
    Resolves paths for container vs host execution

    Automatically detects if running in a container and adjusts
    paths for sessions, logs, and other artifacts.
    """

    def __init__(self, workspace_root: Optional[str] = None):
        """
        Initialize path resolver.

        Args:
            workspace_root: Override workspace root path
        """
        self.is_container = self._detect_container()
        self.workspace_root = workspace_root or self._get_workspace_root()

    def _detect_container(self) -> bool:
        """
        Detect if running inside a container.

        Checks for:
        - /.dockerenv file (Docker)
        - /run/.containerenv (Podman)
        - Environment variable CONTAINER=true
        """
        # Check for Docker
        if os.path.exists('/.dockerenv'):
            return True

        # Check for Podman
        if os.path.exists('/run/.containerenv'):
            return True

        # Check environment variable
        if os.environ.get('CONTAINER', '').lower() == 'true':
            return True

        # Check for typical container cgroup patterns
        try:
            with open('/proc/1/cgroup', 'r') as f:
                content = f.read()
                if 'docker' in content or 'lxc' in content or 'kubepods' in content:
                    return True
        except (FileNotFoundError, PermissionError):
            pass

        return False

    def _get_workspace_root(self) -> str:
        """Get workspace root directory"""
        if self.is_container:
            # In container, use /workspace as root
            return os.environ.get('WORKSPACE_ROOT', '/workspace')
        else:
            # On host, use current working directory or parent
            cwd = os.getcwd()

            # If we're in the tossing_system directory, use it
            if cwd.endswith('tossing_system'):
                return cwd

            # Otherwise try to find it
            for parent_level in range(5):  # Search up to 5 levels up
                test_path = os.path.join(cwd, *(['..'] * parent_level), 'tossing_system')
                if os.path.exists(test_path):
                    return os.path.abspath(test_path)

            # Fallback to cwd
            return cwd

    def resolve_session_dir(self, experiment_id: Optional[str] = None) -> str:
        """
        Resolve sessions directory path.

        Args:
            experiment_id: Optional experiment ID to include in path

        Returns:
            Absolute path to sessions directory
        """
        if self.is_container:
            base = "/workspace/sessions/experiments"
        else:
            base = os.path.join(self.workspace_root, "sessions", "experiments")

        if experiment_id:
            return os.path.join(base, experiment_id)
        return base

    def resolve_config_dir(self) -> str:
        """Resolve experiment configs directory"""
        if self.is_container:
            return "/workspace/experiments"
        else:
            return os.path.join(self.workspace_root, "experiments")

    def resolve_analysis_dir(self, experiment_id: str) -> str:
        """Resolve analysis output directory for an experiment"""
        session_dir = self.resolve_session_dir(experiment_id)
        return os.path.join(session_dir, "analysis")

    def get_ros_master_uri(self) -> str:
        """Get ROS Master URI appropriate for environment"""
        # Check environment first
        if 'ROS_MASTER_URI' in os.environ:
            return os.environ['ROS_MASTER_URI']

        if self.is_container:
            # In container, assume ROS master is on host or named service
            return os.environ.get('ROS_MASTER_URI', 'http://rosmaster:11311')
        else:
            # On host, use localhost
            return 'http://localhost:11311'

    def get_environment_info(self) -> Dict[str, str]:
        """Get information about current environment"""
        return {
            'is_container': self.is_container,
            'workspace_root': self.workspace_root,
            'sessions_dir': self.resolve_session_dir(),
            'configs_dir': self.resolve_config_dir(),
            'ros_master_uri': self.get_ros_master_uri(),
        }

    def print_environment_info(self):
        """Print environment information"""
        info = self.get_environment_info()

        print("\n" + "=" * 70)
        print("ENVIRONMENT INFORMATION")
        print("=" * 70)
        print(f"Execution Environment: {'Container' if info['is_container'] else 'Host'}")
        print(f"Workspace Root: {info['workspace_root']}")
        print(f"Sessions Directory: {info['sessions_dir']}")
        print(f"Configs Directory: {info['configs_dir']}")
        print(f"ROS Master URI: {info['ros_master_uri']}")
        print("=" * 70 + "\n")


# Global singleton instance
_resolver = None


def get_resolver() -> ContainerPathResolver:
    """Get global path resolver instance"""
    global _resolver
    if _resolver is None:
        _resolver = ContainerPathResolver()
    return _resolver


def is_container() -> bool:
    """Check if running in container"""
    return get_resolver().is_container


def resolve_session_dir(experiment_id: Optional[str] = None) -> str:
    """Convenience function to resolve session directory"""
    return get_resolver().resolve_session_dir(experiment_id)


def resolve_config_dir() -> str:
    """Convenience function to resolve config directory"""
    return get_resolver().resolve_config_dir()


if __name__ == '__main__':
    # Test script
    resolver = ContainerPathResolver()
    resolver.print_environment_info()
