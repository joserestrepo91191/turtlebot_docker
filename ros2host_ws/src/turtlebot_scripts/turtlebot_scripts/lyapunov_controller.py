# lyapunov_controller.py
import numpy as np

class LyapunovController:
    def __init__(self, k_rho=20.0, k_alpha=1.5, k_beta=-0.6): # k_rho=0.8
        """
        k_rho: ganancia para el error de distancia.
        k_alpha: ganancia para el error de dirección.
        k_beta: ganancia para el error de orientación.
        """
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta

    def compute_control(self, current_x, current_y, current_yaw, target_x, target_y, target_yaw=None):
        """
        Calcula las velocidades de control usando un enfoque basado en Lyapunov.
        Este controlador está diseñado para robots diferenciales (movimiento en el plano).

        Parámetros:
            current_x, current_y, current_yaw: posición y orientación actual del robot.
            target_x, target_y: posición objetivo.
            target_yaw: orientación deseada en el objetivo (opcional).

        Retorna:
            v: velocidad lineal.
            w: velocidad angular.
            rho: error de distancia.
            alpha: error angular (dirección a la meta respecto a la orientación actual).
            beta: error de orientación.
        """
        # Error en posición
        dx = target_x - current_x
        dy = target_y - current_y
        rho = np.sqrt(dx**2 + dy**2)
        
        # Ángulo entre la posición actual y la meta
        theta_target = np.arctan2(dy, dx)
        alpha = theta_target - current_yaw
        # Normalizar alpha a [-pi, pi]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        # Cálculo de beta: diferencia entre la orientación deseada y la actual, compensando alpha
        if target_yaw is None:
            beta = - current_yaw - alpha
        else:
            beta = target_yaw - current_yaw - alpha
        beta = np.arctan2(np.sin(beta), np.cos(beta))
        
        # Ley de control
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        return v, w, rho, alpha, beta