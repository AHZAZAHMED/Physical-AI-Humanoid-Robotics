#!/usr/bin/env python3
"""
Development server runner for RAG Chatbot UI Integration.

This script starts both the backend API and frontend servers simultaneously
for local development and testing of the RAG chatbot functionality.
"""

import os
import sys
import subprocess
import threading
import time
import signal
import requests
from pathlib import Path


class DevServerRunner:
    def __init__(self):
        self.backend_process = None
        self.frontend_process = None
        self.running = False

    def check_port(self, port):
        """Check if a port is available."""
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            return s.connect_ex(('localhost', port)) != 0

    def start_backend(self):
        """Start the backend FastAPI server."""
        print("🚀 Starting backend server...")

        backend_dir = Path("./backend")
        if not backend_dir.exists():
            print("❌ Backend directory not found!")
            return False

        # Change to backend directory
        os.chdir(backend_dir)

        # Start the backend server
        self.backend_process = subprocess.Popen([
            sys.executable, "-m", "uvicorn",
            "api:app",
            "--host", "0.0.0.0",
            "--port", "8000",
            "--reload"
        ])

        # Change back to main directory
        os.chdir("..")

        # Wait a bit for the server to start
        time.sleep(3)

        # Test if backend is running
        try:
            response = requests.get("http://localhost:8000/", timeout=5)
            if response.status_code == 200:
                print("✅ Backend server started successfully on http://localhost:8000")
                return True
            else:
                print(f"❌ Backend server returned status code: {response.status_code}")
                return False
        except requests.exceptions.ConnectionError:
            print("❌ Backend server failed to start")
            return False

    def start_frontend(self):
        """Start the frontend Docusaurus server."""
        print("🚀 Starting frontend server...")

        frontend_dir = Path("./physical-ai-textbook")
        if not frontend_dir.exists():
            print("❌ Frontend directory not found!")
            return False

        # Change to frontend directory
        os.chdir(frontend_dir)

        # Install dependencies if needed
        print("📦 Installing frontend dependencies...")
        try:
            subprocess.run([sys.executable, "-m", "pip", "install", "nodeenv"], check=True)

            # Check if node is available
            result = subprocess.run(["which", "node"], capture_output=True, text=True)
            if result.returncode != 0:
                # Install node if not available
                subprocess.run([sys.executable, "-m", "nodeenv", "node_env"], check=True)
                os.environ["PATH"] = f"{os.getcwd()}/node_env/bin:{os.environ['PATH']}"

        except subprocess.CalledProcessError:
            print("⚠️  Could not install node dependencies. Make sure Node.js is installed.")

        # Start the Docusaurus server
        self.frontend_process = subprocess.Popen([
            "npx", "docusaurus", "start"
        ], shell=True)  # Use shell=True to handle npx properly

        # Change back to main directory
        os.chdir("..")

        # Wait a bit for the server to start
        time.sleep(5)

        # Test if frontend is running
        try:
            response = requests.get("http://localhost:3000/", timeout=10)
            if response.status_code == 200:
                print("✅ Frontend server started successfully on http://localhost:3000")
                return True
            else:
                print(f"❌ Frontend server returned status code: {response.status_code}")
                return False
        except requests.exceptions.ConnectionError:
            print("❌ Frontend server failed to start")
            return False

    def health_check(self):
        """Perform health checks on both servers."""
        print("\n🔍 Performing health checks...")

        # Check backend
        try:
            response = requests.get("http://localhost:8000/api/health", timeout=5)
            if response.status_code == 200:
                print("✅ Backend health check: OK")
            else:
                print(f"❌ Backend health check failed: {response.status_code}")
        except requests.exceptions.ConnectionError:
            print("❌ Backend health check failed: Cannot connect")

        # Check frontend
        try:
            response = requests.get("http://localhost:3000/", timeout=5)
            if response.status_code == 200:
                print("✅ Frontend health check: OK")
            else:
                print(f"❌ Frontend health check failed: {response.status_code}")
        except requests.exceptions.ConnectionError:
            print("❌ Frontend health check failed: Cannot connect")

    def test_chat_functionality(self):
        """Test the chat functionality."""
        print("\n💬 Testing chat functionality...")

        try:
            # Test a simple query
            test_data = {
                "query": "What is Physical AI and Humanoid Robotics?",
                "sessionId": "test-session-dev-server"
            }

            response = requests.post(
                "http://localhost:8000/api/chat",
                json=test_data,
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                print("✅ Chat API test: SUCCESS")
                print(f"   Response preview: {result.get('response', '')[:100]}...")
                if result.get('sources'):
                    print(f"   Sources found: {len(result['sources'])}")
                return True
            else:
                print(f"❌ Chat API test failed: {response.status_code}")
                print(f"   Response: {response.text}")
                return False
        except requests.exceptions.ConnectionError:
            print("❌ Chat API test failed: Cannot connect to backend")
            return False
        except Exception as e:
            print(f"❌ Chat API test failed with error: {e}")
            return False

    def run_servers(self):
        """Run both servers and perform tests."""
        print("🚀 Starting RAG Chatbot Development Servers")
        print("=" * 50)

        # Check if ports are available
        if not self.check_port(8000):
            print("❌ Port 8000 (backend) is already in use!")
            return False

        if not self.check_port(3000):
            print("❌ Port 3000 (frontend) is already in use!")
            return False

        print("Ports are available. Starting servers...\n")

        # Start backend first
        backend_ok = self.start_backend()
        if not backend_ok:
            print("❌ Failed to start backend server")
            return False

        # Start frontend
        frontend_ok = self.start_frontend()
        if not frontend_ok:
            print("❌ Failed to start frontend server")
            # Stop backend before exiting
            if self.backend_process:
                self.backend_process.terminate()
            return False

        # Perform health checks
        self.health_check()

        # Test chat functionality
        self.test_chat_functionality()

        print("\n" + "=" * 50)
        print("🎯 Servers are running!")
        print("   Backend: http://localhost:8000")
        print("   Frontend: http://localhost:3000")
        print("   Chat API: http://localhost:8000/api/chat")
        print("\n💡 The chatbot should be available as a floating icon on the frontend")
        print("   Press Ctrl+C to stop both servers\n")

        self.running = True

        # Keep the script running
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Shutting down servers...")
            self.stop_servers()

        return True

    def stop_servers(self):
        """Stop both servers."""
        print("Stopping servers...")

        if self.backend_process:
            self.backend_process.terminate()
            self.backend_process.wait()
            print("✅ Backend server stopped")

        if self.frontend_process:
            self.frontend_process.terminate()
            self.frontend_process.wait()
            print("✅ Frontend server stopped")

        self.running = False


def main():
    """Main function to run the development servers."""
    if not os.path.exists("backend") or not os.path.exists("physical-ai-textbook"):
        print("❌ Project directories not found!")
        print("Make sure you're running this script from the project root directory.")
        sys.exit(1)

    runner = DevServerRunner()

    try:
        success = runner.run_servers()
        if not success:
            sys.exit(1)
    except Exception as e:
        print(f"❌ Error running servers: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()