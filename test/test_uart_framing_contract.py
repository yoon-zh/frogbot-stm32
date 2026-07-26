from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
USART_SOURCE = ROOT / "Core/Src/platform/usart.c"


def function_body(source: str, name: str) -> str:
    match = re.search(rf"(?m)^[ \t]*void\s+{name}\s*\([^)]*\)\s*\{{", source)
    assert match, f"{name}() is missing"
    index = match.end()
    depth = 1
    while index < len(source) and depth:
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
        index += 1
    assert depth == 0, f"{name}() body is not balanced"
    return source[match.end() : index - 1]


def test_dma_completion_only_marks_overflow_and_resynchronises() -> None:
    source = USART_SOURCE.read_text(encoding="utf-8", errors="ignore")
    callback = function_body(source, "HAL_UART_RxCpltCallback")

    assert "uart1_rx_overflow = 1u;" in callback
    assert "memcpy" not in callback
    assert "UART1_RX_Size" not in callback
    assert "new_serial_data_received" not in callback
    assert "HAL_UART_Receive_DMA" not in callback


def test_idle_is_the_only_variable_length_packet_publisher() -> None:
    source = USART_SOURCE.read_text(encoding="utf-8", errors="ignore")
    handler = function_body(source, "USART1_IRQHandler")

    capture_index = handler.index(
        "uint16_t dmaRemaining = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);"
    )
    stop_index = handler.index("HAL_UART_DMAStop(&huart1);")
    length_index = handler.index("uint16_t dataLength = DMA_RX_BUF_SIZE - dmaRemaining;")

    assert capture_index < stop_index < length_index
    assert "uart1_rx_overflow == 0u" in handler
    assert "dataLength > 0u" in handler
    assert "dataLength < UART_RX_BUF_SIZE" in handler
    assert "memcpy(UART1_RX_Buffer, DMA_RX_Buffer, dataLength);" in handler
    assert "UART1_RX_Size = dataLength;" in handler
    assert "new_serial_data_received = 1;" in handler
    assert "uart1_rx_overflow = 0u;" in handler
