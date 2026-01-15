"""
Test query generation module for RAG retrieval validation.

This module creates test queries across multiple categories
(keyword, semantic, section-specific) for validation purposes.
"""

import random
from typing import List, Tuple
from .config import config


class QueryGenerator:
    """Creates test queries across different categories for validation."""

    def __init__(self):
        # Predefined query templates for different categories
        self.keyword_queries = [
            "Physical AI definition",
            "Humanoid Robotics overview",
            "ROS 2 architecture",
            "Gazebo simulation",
            "Isaac Sim setup",
            "VSLAM tutorial",
            "Robot kinematics",
            "Navigation system",
            "Machine learning in robotics",
            "Sensor fusion techniques"
        ]

        self.semantic_queries = [
            "How does physical AI differ from traditional AI?",
            "What are the key challenges in humanoid robotics?",
            "Explain the ROS 2 communication patterns",
            "Describe the process of setting up a robot simulation environment",
            "How does visual SLAM work in robotics?",
            "What are the main components of a humanoid robot?",
            "Explain the difference between forward and inverse kinematics",
            "How does the navigation stack work in ROS 2?",
            "What are the best practices for robot perception?",
            "How do you implement robot control systems?"
        ]

        self.section_specific_queries = [
            "Introduction to Physical AI concepts",
            "Chapter 2: ROS 2 Fundamentals",
            "Section on Gazebo physics simulation",
            "Isaac Sim installation guide",
            "Navigation 2 (Nav2) setup",
            "Humanoid robot kinematics equations",
            "Sensor simulation in robotics",
            "Best practices for robot control",
            "Overview of Isaac ROS packages",
            "Cognitive planning for humanoid robots"
        ]

    def generate_keyword_queries(self, count: int = 5) -> List[str]:
        """
        Generate keyword-based queries.

        Args:
            count: Number of queries to generate

        Returns:
            List[str]: Generated keyword queries
        """
        if count > len(self.keyword_queries):
            # If more queries are needed, duplicate and modify existing ones
            all_queries = self.keyword_queries[:]
            while len(all_queries) < count:
                for query in self.keyword_queries:
                    modified_query = f"{query} concepts" if "concepts" not in query else f"advanced {query}"
                    all_queries.append(modified_query)
                    if len(all_queries) >= count:
                        break
            return all_queries[:count]

        return random.sample(self.keyword_queries, count)

    def generate_semantic_queries(self, count: int = 5) -> List[str]:
        """
        Generate semantic queries.

        Args:
            count: Number of queries to generate

        Returns:
            List[str]: Generated semantic queries
        """
        if count > len(self.semantic_queries):
            # If more queries are needed, duplicate and modify existing ones
            all_queries = self.semantic_queries[:]
            while len(all_queries) < count:
                for query in self.semantic_queries:
                    modified_query = f"What is {query.lower().replace('how does', '').replace('explain', '').strip()}?" if not query.endswith('?') else query
                    all_queries.append(modified_query)
                    if len(all_queries) >= count:
                        break
            return all_queries[:count]

        return random.sample(self.semantic_queries, count)

    def generate_section_specific_queries(self, count: int = 5) -> List[str]:
        """
        Generate section-specific queries.

        Args:
            count: Number of queries to generate

        Returns:
            List[str]: Generated section-specific queries
        """
        if count > len(self.section_specific_queries):
            # If more queries are needed, duplicate and modify existing ones
            all_queries = self.section_specific_queries[:]
            while len(all_queries) < count:
                for query in self.section_specific_queries:
                    modified_query = query.replace("Section", "Chapter") if "Section" in query else f"Chapter about {query.lower().replace('section on', '').strip()}"
                    all_queries.append(modified_query)
                    if len(all_queries) >= count:
                        break
            return all_queries[:count]

        return random.sample(self.section_specific_queries, count)

    def generate_test_queries(self, query_counts: dict = None) -> List[Tuple[str, str]]:
        """
        Generate test queries across all categories.

        Args:
            query_counts: Dictionary with counts for each category (keyword, semantic, section-specific)

        Returns:
            List[Tuple[str, str]]: List of (query, category) tuples
        """
        if query_counts is None:
            query_counts = {
                'keyword': 3,
                'semantic': 3,
                'section-specific': 2
            }

        queries = []

        # Generate keyword queries
        keyword_queries = self.generate_keyword_queries(query_counts.get('keyword', 0))
        for query in keyword_queries:
            queries.append((query, 'keyword'))

        # Generate semantic queries
        semantic_queries = self.generate_semantic_queries(query_counts.get('semantic', 0))
        for query in semantic_queries:
            queries.append((query, 'semantic'))

        # Generate section-specific queries
        section_queries = self.generate_section_specific_queries(query_counts.get('section-specific', 0))
        for query in section_queries:
            queries.append((query, 'section-specific'))

        return queries

    def add_custom_query(self, query: str, category: str):
        """
        Add a custom query to the appropriate category list.

        Args:
            query: The query string to add
            category: The category ('keyword', 'semantic', 'section-specific')
        """
        if category == 'keyword':
            if query not in self.keyword_queries:
                self.keyword_queries.append(query)
        elif category == 'semantic':
            if query not in self.semantic_queries:
                self.semantic_queries.append(query)
        elif category == 'section-specific':
            if query not in self.section_specific_queries:
                self.section_specific_queries.append(query)
        else:
            raise ValueError(f"Unknown category: {category}")

    def get_sample_queries(self, category: str = None) -> List[str]:
        """
        Get sample queries for a specific category or all categories.

        Args:
            category: Specific category ('keyword', 'semantic', 'section-specific') or None for all

        Returns:
            List[str]: Sample queries for the specified category
        """
        if category == 'keyword':
            return self.keyword_queries
        elif category == 'semantic':
            return self.semantic_queries
        elif category == 'section-specific':
            return self.section_specific_queries
        elif category is None:
            # Return a mix of all categories
            all_queries = []
            all_queries.extend([(q, 'keyword') for q in self.keyword_queries[:3]])
            all_queries.extend([(q, 'semantic') for q in self.semantic_queries[:3]])
            all_queries.extend([(q, 'section-specific') for q in self.section_specific_queries[:2]])
            return [q[0] for q in all_queries]  # Return just the queries without category
        else:
            raise ValueError(f"Unknown category: {category}")