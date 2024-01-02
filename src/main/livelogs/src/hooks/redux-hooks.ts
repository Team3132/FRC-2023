import { useDispatch, type TypedUseSelectorHook, useSelector } from "react-redux"
import type { AppDispatch, RootState } from "../data/store"

export const useAppDispatch: () => AppDispatch = useDispatch
export const useAppSelector: TypedUseSelectorHook<RootState> = useSelector