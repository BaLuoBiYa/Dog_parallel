mod game_pad;

use game_pad::GamepadInterface;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[tokio::main]
async fn main() {
    // 1. 创建共享状态 —— Arc<Mutex<>> 等价于 C++ 的 shared_ptr<mutex_wrapper>
    //    Arc = 多所有者（类似 shared_ptr）
    //    Mutex = 互斥锁（类似 std::mutex）
    let gamepad = Arc::new(Mutex::new(GamepadInterface::new().unwrap()));

    // 2. 克隆 Arc（只增加引用计数，不拷贝内部数据）
    let gamepad_bg = Arc::clone(&gamepad);
    //    ^ 等价于 C++ 的 std::shared_ptr<GamepadInterface> gamepad_bg = gamepad;

    // 3. 后台任务：定时刷新手柄
    let bg_handle = tokio::spawn(async move {
        // move 将 gamepad_bg 的所有权移入闭包
        loop {
            {
                let mut pad = gamepad_bg.lock().unwrap();
                //            ^ 这里会阻塞直到拿到锁，等价于 C++ 的 lock_guard
                pad.update();
            }
            // 锁在离开作用域时自动释放 —— RAII

            tokio::time::sleep(Duration::from_millis(10)).await;
            // ^ 只让出当前任务，不阻塞线程
            //   等价于 C++ 的 std::this_thread::sleep_for 但不会阻塞线程
        }
    });

    // 4. 前台任务：打印手柄状态
    let gamepad_fg = Arc::clone(&gamepad);
    let fg_handle = tokio::spawn(async move {
        loop {
            tokio::time::sleep(Duration::from_millis(50)).await;

            // 用块控制锁的作用域，出块自动解锁
            let current_state = {
                let pad = gamepad_fg.lock().unwrap();
                pad.state().clone() // 从 GamepadInterface 中克隆一份快照
            };

            // 打印
            if current_state.connected {
                println!(
                    "🎮 左摇杆: x={:.2}, y={:.2} | A键: {} | 十字键↓: {}",
                    current_state.left_stick_x,
                    current_state.left_stick_y,
                    if current_state.btn_south {
                        "按住"
                    } else {
                        "松开"
                    },
                    if current_state.btn_dpad_down {
                        "▼按下"
                    } else {
                        "  .  "
                    },
                );
            } else {
                print!("⏳");
            }
        }
    });

    // 5. 同时等待后台任务和前台任务
    let _ = tokio::join!(bg_handle, fg_handle);
    // ^ tokio::join! 同时等待多个任务，类似 C++ 中 t1.join(); t2.join();
}
