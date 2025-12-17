/**
 * Authentication Context
 * Provides global authentication state and methods
 */
import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: number;
  email: string;
  full_name: string;
  language_preference: string;
  is_active: boolean;
  is_verified: boolean;
  created_at: string;
  last_login_at: string | null;
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, fullName: string) => Promise<void>;
  logout: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Check for existing session on mount
  useEffect(() => {
    refreshSession();
  }, []);

  const refreshSession = async () => {
    // Mock implementation - no backend available
    setIsLoading(false);
  };

  const login = async (email: string, password: string) => {
    // Mock implementation - no backend available
    throw new Error('Authentication backend is not available. This is a UI demonstration only.');
  };

  const signup = async (email: string, password: string, fullName: string) => {
    // Mock implementation - no backend available
    throw new Error('Authentication backend is not available. This is a UI demonstration only.');
  };

  const logout = async () => {
    // Mock implementation - no backend available
    localStorage.removeItem('access_token');
    localStorage.removeItem('user');
    setUser(null);
  };

  const value: AuthContextType = {
    user,
    isAuthenticated: !!user,
    isLoading,
    login,
    signup,
    logout,
    refreshSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};
