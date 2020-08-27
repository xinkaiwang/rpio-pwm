
export interface RpioConfig {
  /**
   * by default 20ms
   * typical value 20000=20ms each cycle
   */
  cycle_time_us?: number;
  /**
   * default 10us
   * step_time_us: typical value 10=10us each step 
   * (total steps = 20000/10=2000 steps)
   */
  step_time_us?: number;
  /**
   * 0=PWM (default)
   * 1=PCM
   */
  delay_hw?: 0 | 1;
  /**
   * 0 don't invert (default)
   * 1 invert of !0
   */
  invert?: 0 | 1;
}

export interface RpioPwm {
  set_width(gpio: number): void;
  release(): void;
}

export interface RpioDmaChanel {
  create_pwm(gpio: number): RpioPwm,
  shutdown(): void;
}

export function create_dma_channel(ch: number, cfg?: RpioConfig): RpioDmaChanel;

export function host_is_model_pi4() : boolean;

// need keep sync with LogLevel in dma.h
interface ILogLevel {
  fatal: number,
  error: number,
  warning: number,
  info: number,
  debug: number,
}

export function set_log_level(logLevel: ILogLevel): void

export const logLevel: ILogLevel;

