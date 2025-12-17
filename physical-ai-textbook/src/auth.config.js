// Authentication configuration for Better-Auth
// This file will be used when Better-Auth is installed

import { betterAuth } from "better-auth";
import { nextCookies } from "@better-auth/next-js";

// Create the auth client with user metadata for software/hardware background
export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
  },
  socialProviders: {
    // Add social providers as needed
  },
  // Add custom user fields for software and hardware background
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: true, // Required for registration
      },
      hardwareBackground: {
        type: "string",
        required: true, // Required for registration
      },
    },
  },
  // Custom registration flow with questionnaire
  account: {
    account: {
      required: true,
    },
  },
});

// Next.js cookies adapter
export const cookies = nextCookies();