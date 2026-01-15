# Book Content RAG Agent API Documentation

## Overview

This API provides a Retrieval-Augmented Generation (RAG) system that processes user queries against book content and generates grounded, educational responses. The system uses vector similarity search to find relevant book content and an AI agent to generate instructional responses based on the retrieved information.

## API Endpoints

### Query Processing
- **POST** `/api/v1/query` - Process user queries through the RAG pipeline

### Health Check
- **GET** `/api/v1/health` - Check service and dependency health

### Statistics
- **GET** `/api/v1/stats` - Retrieve usage statistics and performance metrics
- **GET** `/api/v1/stats/detailed` - Retrieve detailed statistics with recent query performance

### Configuration
- **GET** `/api/v1/config` - Get current system configuration
- **PUT** `/api/v1/config` - Update system configuration

## Authentication

All endpoints require authentication using a Bearer token in the Authorization header.

## Rate Limiting

The API implements rate limiting to ensure fair usage and system stability.