
export interface RpioConfig {
  /**
   * by default 20ms
   */
  cycle_time_us?: number;
  /**
   * default 10us
   */
  step_time_us?: number;
  /**
   * default 0 (PWM)
   */
  delay_hw?: number;
  /**
   * default 0 (don't invert)
   */
  invert?: number;
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

