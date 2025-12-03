class ServoIntegratorController:
    """
    Implementa el controlador SERVO INTEGRADOR discreto de orden 3:
    
        u[k] = a1*u[k-1] + a2*u[k-2] + a3*u[k-3]
             + b0*r[k] + b1*r[k-1] + b2*r[k-2]
             + c0*y[k] + c1*y[k-1] + c2*y[k-2]
             
    Igual estructura de uso que el PIDController:
        servo = ServoIntegratorController(A,B,C)
        u = servo.compute(r, y)
    """

    def __init__(self, a, b, c):
        self.a = a  # coeficientes de u[k-i]
        self.b = b  # coeficientes de r[k-i]
        self.c = c  # coeficientes de y[k-i]

        # Memorias internas
        self.u_hist = [0.0, 0.0, 0.0]
        self.r_hist = [0.0, 0.0, 0.0]
        self.y_hist = [0.0, 0.0, 0.0]

        # Para log
        self.last_output = 0.0

    def compute(self, r, y):
        """Computa u[k] basado en r[k] y y[k]."""

        # Insertar valores actuales en historial
        self.r_hist = [r] + self.r_hist[:2]
        self.y_hist = [y] + self.y_hist[:2]

        # Calcular control
        u = (
            self.a[0]*self.u_hist[0] +
            self.a[1]*self.u_hist[1] +
            self.a[2]*self.u_hist[2] +
            self.b[0]*self.r_hist[0] +
            self.b[1]*self.r_hist[1] +
            self.b[2]*self.r_hist[2] +
            self.c[0]*self.y_hist[0] +
            self.c[1]*self.y_hist[1] +
            self.c[2]*self.y_hist[2]
        )

        # Actualizar memoria
        self.u_hist = [u] + self.u_hist[:2]
        self.last_output = u

        return u
